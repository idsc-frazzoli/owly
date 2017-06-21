// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinCurvatureGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.Se2WrapGoalManagerExt;
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

/** (x,y,theta) */
class Se2IterateGlcAnyCircleWrapDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(6);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(5, 5, 20 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 12);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    parameters.printResolution();
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), parameters.getResolutionInt());
    CoordinateWrap coordinateWrap = new Se2Wrap(Tensors.vector(1, 1, 1));
    Se2MinCurvatureGoalManager se2GoalManager = new Se2MinCurvatureGoalManager( //
        Tensors.vector(3, 0), RealScalar.of(1.5 * Math.PI), // east
        DoubleScalar.of(.1), Se2Utils.DEGREE(30));
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
    DebugUtils.nodeAmountCompare(trajectoryPlanner);
    System.out.println("After " + iters + " iterations");
    System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
    owlyFrame.setGlc(trajectoryPlanner);
    // ---
    List<Tensor> goalList = new ArrayList<>();
    goalList.add(Tensors.vector(0, -3, Math.PI));// South
    goalList.add(Tensors.vector(-3, 0, 0.5 * Math.PI));// West
    goalList.add(Tensors.vector(0, 3, 0)); // North
    goalList.add(Tensors.vector(3, 0, -0.5 * Math.PI)); // East
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
      int index = iter % 4;
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
      StateTime newRootState = trajectory.get(5);
      int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      parameters.increaseDepthLimit(increment);
      Thread.sleep(delay.number().intValue() / 2);
      Se2MinCurvatureGoalManager se2GoalManager2 = new Se2MinCurvatureGoalManager( //
          Tensors.of(goalList.get(index).Get(0), goalList.get(index).Get(1)), goalList.get(index).Get(2), //
          DoubleScalar.of(0.5), Se2Utils.DEGREE(30));
      Se2WrapGoalManagerExt se2WrapGoalManager2 = new Se2WrapGoalManagerExt( //
          coordinateWrap, //
          se2GoalManager2);
      // --
      goalFound = trajectoryPlanner.changeGoal(se2WrapGoalManager2.getGoalInterface());
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
      // owlyFrame.configCoordinateOffset(432, 273);
      iter++;
    }
  }
}
