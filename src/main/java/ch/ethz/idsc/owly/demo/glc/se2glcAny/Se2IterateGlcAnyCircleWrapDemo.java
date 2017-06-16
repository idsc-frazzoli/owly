// code by jl
package ch.ethz.idsc.owly.demo.glc.se2glcAny;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.glc.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.glc.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.glc.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.glc.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.glc.se2.Se2WrapGoalManager;
import ch.ethz.idsc.owly.demo.glc.se2glc.Se2Parameters;
import ch.ethz.idsc.owly.demo.glc.tn.IdentityWrap;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.Expand;
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
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 12);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    parameters.printResolution();
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), parameters.getResolutionInt());
    final CoordinateWrap identity = new IdentityWrap();
    CoordinateWrap coordinateWrap;
    coordinateWrap = new Se2Wrap(Tensors.vector(1, 1, 1));
    coordinateWrap = identity;
    Se2WrapGoalManager se2GoalManager = new Se2WrapGoalManager( //
        coordinateWrap, //
        Tensors.vector(3, 0, 1.5 * Math.PI), // east
        DoubleScalar.of(.25));
    // Se2MinCurvatureGoalManager se2GoalManager = new Se2MinCurvatureGoalManager( //
    // Tensors.vector(3, 0), RealScalar.of(1.5 * Math.PI), // east
    // DoubleScalar.of(.1), Se2Utils.DEGREE(30));
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
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 3, 0));
    OwlyFrame owlyFrame = Gui.start();
    int iters = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    DebugUtils.nodeAmountCompare(trajectoryPlanner);
    System.out.println("After " + iters + " iterations");
    List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
    Scalar toc = RealScalar.of(System.nanoTime());
    System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
    Trajectories.print(trajectory);
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
    while (owlyFrame.jFrame.isVisible()) {
      Scalar delay = RealScalar.of(0);
      Thread.sleep(1000);
      tic = RealScalar.of(System.nanoTime());
      int index = iter % 4;
      // --
      StateTime newRootState = trajectory.get(5);
      int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      DebugUtils.nodeAmountCompare(trajectoryPlanner);
      parameters.increaseDepthLimit(increment);
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(delay.number().intValue() / 2);
      List<Integer> positionIndex = new ArrayList<Integer>();
      positionIndex.add(1);
      positionIndex.add(2);
      // Se2MinCurvatureGoalManager se2GoalManager2 = new Se2MinCurvatureGoalManager( //
      // goalList.get(index).get(positionIndex), goalList.get(index).get(3), //
      // DoubleScalar.of(0.5), Se2Utils.DEGREE(30));
      Se2WrapGoalManager se2GoalManager2 = new Se2WrapGoalManager( //
          coordinateWrap, //
          goalList.get(index), // east
          DoubleScalar.of(.25));
      // --
      goalFound = trajectoryPlanner.changeGoal(se2GoalManager2.getGoalInterface());
      DebugUtils.nodeAmountCompare(trajectoryPlanner);
      owlyFrame.setGlc(trajectoryPlanner);
      Thread.sleep(delay.number().intValue() / 2);
      // --
      if (!goalFound)
        expandIter = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      // Bug changes with goalmanager (time of occurence)
      // ---
      toc = RealScalar.of(System.nanoTime());
      trajectory = trajectoryPlanner.getPathFromRootToGoal();
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
