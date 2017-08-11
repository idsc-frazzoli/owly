// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2DefaultGoalManagerExt;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.SimpleAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** (x,y,theta) */
class Se2IterateSimpleGlcAnyCircleDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(6);
    Scalar timeScale = RealScalar.of(6);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    Scalar circleRadius = RealScalar.of(3);
    Scalar goalAngle = RealScalar.of(0);
    Tensor goal = Tensors.of(Cos.of(goalAngle).multiply(circleRadius), //
        Sin.of(goalAngle).multiply(circleRadius), goalAngle.subtract(RealScalar.of(Math.PI * 0.5)));
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), RotationUtils.DEGREE(15));
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    System.out.println("1/Domainsize=" + parameters.getEta());
    parameters.printResolution();
    // Se2Controls uses Se2StateSpaceModel
    Collection<Flow> controls = Se2Controls.createControls(RotationUtils.DEGREE(45), parameters.getResolutionInt());
    Se2DefaultGoalManagerExt se2GoalManager = new Se2DefaultGoalManagerExt(goal, radiusVector);
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(1, 1, Double.POSITIVE_INFINITY)) //
                , new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(5, 5, Double.POSITIVE_INFINITY))) //
            // ,new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)) //
            // ,new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4)) //
            )));
    // ---
    long tic = System.nanoTime();
    SimpleAnyTrajectoryPlanner trajectoryPlanner = new SimpleAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 3, 0));
    int iters = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    DebugUtils.nodeAmountCompare(trajectoryPlanner);
    System.out.println("After " + iters + " iterations");
    long toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to plan");
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.setGlc(trajectoryPlanner);
    // ---
    // --
    int iter = 1;
    Scalar timeSum = RealScalar.of(0);
    while (owlyFrame.jFrame.isVisible()) {
      Thread.sleep(000);
      tic = System.nanoTime();
      List<StateTime> trajectory = null;
      {
        Optional<GlcNode> optional = trajectoryPlanner.getBestOrElsePeek();
        if (optional.isPresent()) {
          trajectory = GlcNodes.getPathFromRootTo(optional.get());
        } else {
          throw new RuntimeException();
        }
      }
      StateTime newRootState = trajectory.get(2);
      int increment = trajectoryPlanner.switchRootToState(newRootState.state());
      parameters.increaseDepthLimit(increment);
      boolean foundGoal = Se2CircleAnyGoalSwitch.switchToNextCircularGoal(trajectoryPlanner, iter);
      int expandIter = 0;
      if (!foundGoal)
        expandIter = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      StateTimeTrajectories.print(trajectory);
      // ---
      toc = System.nanoTime();
      timeSum = RealScalar.of(toc - tic).multiply(RealScalar.of(1e-9)).add(timeSum);
      System.out.println((iter + 1) + " iteration: " + (toc - tic) * 1e-9);
      System.out.println("Average: " + timeSum.divide(RealScalar.of(iter + 1)));
      System.out.println("After root switch needed " + expandIter + " iterations");
      System.out.println("*****Finished*****");
      System.out.println("");
      owlyFrame.setGlc(trajectoryPlanner);
      // owlyFrame.configCoordinateOffset(432, 273);
      iter++;
    }
  }
}
