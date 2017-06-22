// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnGoalManager;
import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Mod;

class R2GlcAnyDemo {
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
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(new EulerIntegrator(), parameters.getdtMax(), //
        parameters.getTrajectorySize());
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(5, 5), DoubleScalar.of(.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(new EmptyRegion()));
    // TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery( //
    // new TimeInvariantRegion(new R2Bubbles()));
    // ---
    AnyPlannerInterface trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.switchRootToState(Tensors.vector(0, 0));
    Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    OwlyFrame owlyFrame = Gui.start();
    Tensor goal = Tensors.vector(6, 6);
    for (int iter = 0; iter < 20; iter++) {
      long tic = System.nanoTime();
      goal = goal.add(Tensors.vector(1, 1));
      goal.set(Mod.function(RealScalar.of(5)), 0);
      goal.set(Mod.function(RealScalar.of(5)), 1);
      RnGoalManager rnGoal2 = new RnGoalManager(goal, DoubleScalar.of(.25));
      List<StateTime> trajectory = trajectoryPlanner.trajectoryToBest();
      if (trajectory != null) {
        StateTime newRootState = trajectory.get(trajectory.size() > 1 ? 1 : 0);
        // ---
        int increment = trajectoryPlanner.switchRootToState(newRootState.x());
        parameters.increaseDepthLimit(increment);
      } else {
        throw new RuntimeException();
      }
      System.out.println("Switching to Goal:" + goal);
      trajectoryPlanner.changeToGoal(rnGoal2);
      int iters2 = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      // ---
      long toc = System.nanoTime();
      Trajectories.print(trajectory);
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
      // owlyFrame.configCoordinateOffset(150 - iter * 30, 450 + iter * 30);
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
  }
}
