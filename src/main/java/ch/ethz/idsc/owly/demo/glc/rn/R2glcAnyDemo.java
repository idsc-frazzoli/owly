// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.wrap.Parameters;
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

class R2glcAnyDemo {
  public static void main(String[] args) {
    RationalScalar resolution = (RationalScalar) RealScalar.of(4);
    Scalar timeScale = RealScalar.of(10);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(3, 3);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(new EulerIntegrator(), RationalScalar.of(1, 5), 5);
    Collection<Flow> controls = R2Controls.createControls(parameters.getResolution().number().intValue());
    RnGoalManager rnGoal = new RnGoalManager(Tensors.vector(5, 5), DoubleScalar.of(.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(new EmptyRegion()));
    // TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery( //
    // new TimeInvariantRegion(new R2Bubbles()));
    // ---
    AnyTrajectoryPlanner trajectoryPlanner = new AnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, rnGoal, rnGoal, obstacleQuery);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.setGlc(trajectoryPlanner);
    for (int iter = 0; iter < 3; iter++) {
      long tic = System.nanoTime();
      RnGoalManager rnGoal2 = new RnGoalManager(Tensors.vector(6 + iter, 6 + iter), DoubleScalar.of(.25));
      StateTime newRootState = trajectory.get(1);
      // ---
      int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      parameters.increaseDepthLimit(increment);
      trajectoryPlanner.setGoalQuery(rnGoal2, rnGoal2.goalQuery());
      int iters2 = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      trajectory = trajectoryPlanner.getPathFromRootToGoal();
      Trajectories.print(trajectory);
      // ---
      long toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      owlyFrame.setGlc(trajectoryPlanner);
      owlyFrame.configCoordinateOffset(150 - iter * 30, 450 + iter * 30);
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
  }
}
