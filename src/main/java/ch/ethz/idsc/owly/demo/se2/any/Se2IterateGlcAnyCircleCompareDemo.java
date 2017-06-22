// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2MinCurvatureGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
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
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
class Se2IterateGlcAnyCircleCompareDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 12);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), Se2Utils.DEGREE(15));
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    System.out.println("1/Domainsize=" + parameters.getEta());
    parameters.printResolution();
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), parameters.getResolutionInt());
    Se2MinCurvatureGoalManager se2GoalManager = new Se2MinCurvatureGoalManager( //
        Tensors.vector(3, 0, 1.5 * Math.PI), radiusVector);
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(1, 1, Double.POSITIVE_INFINITY)), //
                new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(5, 5, Double.POSITIVE_INFINITY))), //
                new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4)) //
            )));
    List<Tensor> goalList = new ArrayList<>();
    goalList.add(Tensors.vector(0, -3, Math.PI));// South
    goalList.add(Tensors.vector(-3, 0, 0.5 * Math.PI));// West
    goalList.add(Tensors.vector(0, 3, 0)); // North
    goalList.add(Tensors.vector(3, 0, -0.5 * Math.PI)); // East
    // ---
    Scalar tic = RealScalar.of(System.nanoTime());
    System.out.println("***ANY***");
    // {
    AnyPlannerInterface anyTrajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    anyTrajectoryPlanner.switchRootToState(Tensors.vector(0, 3, 0));
    int iters = Expand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    Scalar toc = RealScalar.of(System.nanoTime());
    // }
    System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
    OwlyFrame owlyFrameAny = Gui.start();
    owlyFrameAny.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
    // ---
    System.out.println("***DEFAULT**");
    // ---
    OwlyFrame owlyFrameDefault = Gui.start();
    {
      StandardTrajectoryPlanner defaultTrajectoryPlanner = new StandardTrajectoryPlanner( //
          parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
      defaultTrajectoryPlanner.insertRoot(Tensors.vector(0, 3, 0));
      iters = Expand.maxDepth(defaultTrajectoryPlanner, parameters.getDepthLimit());
      DebugUtils.nodeAmountCompare(defaultTrajectoryPlanner);
      System.out.println("After " + iters + " iterations");
      toc = RealScalar.of(System.nanoTime());
      System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
      owlyFrameDefault.setGlc(defaultTrajectoryPlanner);
    }
    // --
    int iter = 0;
    boolean goalFound = false;
    int expandIter = 0;
    System.out.println("****STARTING COMPARISON****");
    while (owlyFrameAny.jFrame.isVisible()) {
      System.out.println("***NEW SET");
      List<StateTime> anyTrajectory = anyTrajectoryPlanner.trajectoryToBest();
      StateTime newRootState = null;
      if (anyTrajectory != null)
        newRootState = anyTrajectory.get(anyTrajectory.size() > 5 ? 5 : 0);
      int index = iter % 4;
      Se2MinCurvatureGoalManager se2GoalManager2 = new Se2MinCurvatureGoalManager( //
          goalList.get(index), radiusVector);
      // --
      System.out.println("***ANY***");
      tic = RealScalar.of(System.nanoTime());
      {
        int increment = anyTrajectoryPlanner.switchRootToState(newRootState.x());
        parameters.increaseDepthLimit(increment);
        // --
        goalFound = anyTrajectoryPlanner.changeToGoal(se2GoalManager2.getGoalInterface());
        // --
        if (!goalFound)
          expandIter = Expand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
        System.out.println("After " + expandIter + " iterations");
        owlyFrameAny.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
        // ---
      }
      toc = RealScalar.of(System.nanoTime());
      System.out.println((iter + 1) + ". iteration took: " + toc.subtract(tic).multiply(RealScalar.of(1e-9)) + "s");
      System.out.println("***DEFAULT***");
      tic = RealScalar.of(System.nanoTime());
      {
        StandardTrajectoryPlanner defaultTrajectoryPlanner = new StandardTrajectoryPlanner( //
            parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager2.getGoalInterface());
        defaultTrajectoryPlanner.insertRoot(newRootState.x());
        iters = Expand.maxDepth(defaultTrajectoryPlanner, parameters.getDepthLimit());
        System.out.println("After " + iters + " iterations");
        toc = RealScalar.of(System.nanoTime());
        System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
        owlyFrameDefault.setGlc(defaultTrajectoryPlanner);
      }
      toc = RealScalar.of(System.nanoTime());
      System.out.println((iter + 1) + ". iteration took: " + toc.subtract(tic).multiply(RealScalar.of(1e-9)) + "s");
      iter++;
    }
  }
}
