// code by jl
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
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
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class R2glcAnyCircleDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(2);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(30, 30);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Scalar circleRadius = RealScalar.of(6);
    Scalar goalAngle = RealScalar.ZERO;
    Tensor goal = Tensors.of(Cos.of(goalAngle), Sin.of(goalAngle)).multiply(circleRadius);
    System.out.println("Goal is: " + goal);
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(new EulerIntegrator(), parameters.getdtMax(), //
        parameters.getTrajectorySize());
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    RnGoalManager rnGoal = new RnGoalManager(goal, DoubleScalar.of(.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(circleRadius).multiply(RealScalar.of(0.5))) //
                , new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(circleRadius).multiply(RealScalar.of(2)))) //
            // ,new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)) //
            // ,new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4)) //
            )));
    // --
    OptimalAnyTrajectoryPlanner trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 1).multiply(circleRadius));
    Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Trajectories.print(trajectory);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.setGlc(trajectoryPlanner);
    for (int iter = 1; iter < 20; iter++) {
      Thread.sleep(3000);
      long tic = System.nanoTime();
      goalAngle = goalAngle.subtract(RealScalar.of(0.1 * Math.PI));
      goal = Tensors.of(Cos.of(goalAngle), Sin.of(goalAngle)).multiply(circleRadius);
      RnGoalManager rnGoal2 = new RnGoalManager(goal, DoubleScalar.of(.25));
      StateTime newRootState = trajectory.get(trajectory.size() > 5 ? 5 : 0);
      // ---
      int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      parameters.increaseDepthLimit(increment);
      System.out.println("Switching to Goal:" + goal);
      trajectoryPlanner.changeGoal(rnGoal2);
      int iters2 = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      trajectory = trajectoryPlanner.getPathFromRootToGoal();
      Trajectories.print(trajectory);
      // ---
      long toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      owlyFrame.setGlc(trajectoryPlanner);
      // owlyFrame.configCoordinateOffset(150 - iter * 30, 450 + iter * 30);
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
  }
}
