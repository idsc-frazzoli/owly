// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.Parameters;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.any.AnyPlannerInterface;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.flow.EulerIntegrator;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.R2Flows;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Mod;

enum R2GlcAnyDemo {
  ;
  public static void main(String[] args) {
    RationalScalar resolution = (RationalScalar) RealScalar.of(4);
    Scalar timeScale = RealScalar.of(2);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(7, 7);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    R2Flows r2Config = new R2Flows(RealScalar.ONE);
    Collection<Flow> controls = r2Config.getFlows(parameters.getResolutionInt());
    GoalInterface rnGoal = RnMinDistSphericalGoalManager.create(Tensors.vector(5, 5), DoubleScalar.of(0.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = EmptyTrajectoryRegionQuery.INSTANCE;
    // TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery( //
    // new TimeInvariantRegion(new R2Bubbles()));
    // ---
    AnyPlannerInterface anyPlannerInterface = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    anyPlannerInterface.switchRootToState(Tensors.vector(0, 0));
    GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
    OwlyFrame owlyFrame = OwlyGui.start();
    Tensor goal = Tensors.vector(6, 6);
    for (int iter = 0; iter < 20; iter++) {
      long tic = System.nanoTime();
      goal = goal.add(Tensors.vector(1, 1));
      goal.set(Mod.function(5), 0);
      goal.set(Mod.function(5), 1);
      GoalInterface rnGoal2 = RnMinDistSphericalGoalManager.create(goal, DoubleScalar.of(0.25));
      List<StateTime> trajectory = anyPlannerInterface.trajectoryToBest();
      if (trajectory != null) {
        StateTime newRootState = trajectory.get(trajectory.size() > 1 ? 1 : 0);
        // ---
        int increment = anyPlannerInterface.switchRootToState(newRootState.state());
        parameters.increaseDepthLimit(increment);
      } else {
        throw new RuntimeException();
      }
      System.out.println("Switching to Goal:" + goal);
      anyPlannerInterface.changeToGoal(rnGoal2);
      int iters2 = GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
      // ---
      long toc = System.nanoTime();
      StateTimeTrajectories.print(trajectory);
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
      // owlyFrame.configCoordinateOffset(150 - iter * 30, 450 + iter * 30);
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
  }
}
