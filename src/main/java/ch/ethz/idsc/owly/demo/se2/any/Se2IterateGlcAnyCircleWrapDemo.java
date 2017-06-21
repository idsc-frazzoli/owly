// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinCurvatureGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.Se2WrapGoalManagerExt;
import ch.ethz.idsc.owly.demo.se2.glc.Se2CircleAnyDemo;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
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

/** (x,y,theta) */
class Se2IterateGlcAnyCircleWrapDemo extends Se2CircleAnyDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(6);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(5, 5, 20 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 12);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    Scalar circleRadius = RealScalar.of(3);
    Scalar goalAngle = RealScalar.of(0);
    Tensor goal = Tensors.of(Cos.of(goalAngle).multiply(circleRadius), //
        Sin.of(goalAngle).multiply(circleRadius), goalAngle.subtract(RealScalar.of(Math.PI * 0.5)));
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), Se2Utils.DEGREE(15));
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    parameters.printResolution();
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), parameters.getResolutionInt());
    CoordinateWrap coordinateWrap = new Se2Wrap(Tensors.vector(1, 1, 1));
    Se2MinCurvatureGoalManager se2GoalManager = new Se2MinCurvatureGoalManager(goal, radiusVector);
    Se2WrapGoalManagerExt se2WrapGoalManagerExt = new Se2WrapGoalManagerExt( //
        coordinateWrap, //
        se2GoalManager);
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(1, 1, Double.POSITIVE_INFINITY)), //
                new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(5, 5, Double.POSITIVE_INFINITY))), //
                new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4)) //
            )));
    // ---
    Scalar tic = RealScalar.of(System.nanoTime());
    OptimalAnyTrajectoryPlanner trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2WrapGoalManagerExt.getGoalInterface());
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 3, 0));
    OwlyFrame owlyFrame = Gui.start();
    Scalar toc = RealScalar.of(System.nanoTime());
    int iters = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
    owlyFrame.setGlc(trajectoryPlanner);
    // ---
    // --
    int iter = 0;
    Scalar timeSum = RealScalar.of(0);
    boolean goalFound = false;
    int expandIter = 0;
    while (iter < 20) {
      // while (owlyFrame.jFrame.isVisible()) {
      Scalar delay = RealScalar.of(0);
      Thread.sleep(1000);
      tic = RealScalar.of(System.nanoTime());
      iter++;
      // --
      List<StateTime> trajectory = null;
      {
        Optional<GlcNode> optional = trajectoryPlanner.getBestOrElsePeek();
        if (optional.isPresent()) {
          trajectory = GlcNodes.getPathFromRootTo(optional.get());
        } else {
          // TODO JONAS maybe change resolution of next iteration --> new planner
          // TODO JONAS write function which finds best merit in Tree
          throw new RuntimeException();
        }
      }
      StateTime newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
      int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      parameters.increaseDepthLimit(increment);
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(delay.number().intValue() / 2);
      // --
      goalFound = switchToNextCircularGoal(trajectoryPlanner, iter);
      Thread.sleep(delay.number().intValue() / 2);
      // --
      if (!goalFound)
        expandIter = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      // ---
      toc = RealScalar.of(System.nanoTime());
      Trajectories.print(trajectory);
      timeSum = toc.subtract(tic).multiply(RealScalar.of(1e-9)).add(timeSum);
      System.out.println((iter + 1) + ". iteration took: " + toc.subtract(tic).multiply(RealScalar.of(1e-9)) + "s");
      System.out.println("Average: " + timeSum.divide(RealScalar.of(iter + 1)));
      System.out.println("After root switch needed " + expandIter + " iterations");
      System.out.println("*****Finished*****");
      System.out.println("");
      owlyFrame.setGlc(trajectoryPlanner);
      DebugUtils.nodeAmountCompare(trajectoryPlanner);
      // owlyFrame.configCoordinateOffset(432, 273);
      iter++;
    }
  }
}
