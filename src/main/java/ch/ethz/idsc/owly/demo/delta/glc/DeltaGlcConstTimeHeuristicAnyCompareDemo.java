// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.data.Stopwatch;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaTrajectoryGoalManager;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryPlannerContainer;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum DeltaGlcConstTimeHeuristicAnyCompareDemo {
  ;
  public static void main(String[] args) throws Exception {
    // -- Quick Planner init
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(10, 1);
    Tensor partitionScale = Tensors.vector(120, 120);
    Stopwatch stopwatchTotal = Stopwatch.started();
    TrajectoryPlannerContainer quickTrajectoryPlannerContainer = DeltaHelper.createGlc(RealScalar.of(-0.02), quickResolution, partitionScale);
    GlcExpand.maxDepth(quickTrajectoryPlannerContainer.getTrajectoryPlanner(), DoubleScalar.POSITIVE_INFINITY.number().intValue());
    System.out.println("QuickPlanner took: " + stopwatchTotal.display_seconds() + "s");
    OwlyFrame quickOwlyFrame = OwlyGui.start();
    quickOwlyFrame.configCoordinateOffset(33, 416);
    quickOwlyFrame.jFrame.setBounds(100, 100, 620, 475);
    quickOwlyFrame.setGlc(quickTrajectoryPlannerContainer.getTrajectoryPlanner());
    Optional<GlcNode> optional = quickTrajectoryPlannerContainer.getTrajectoryPlanner().getBest();
    List<StateTime> quickTrajectory = null;
    if (optional.isPresent()) {
      quickTrajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(quickTrajectory);
    } else {
      throw new RuntimeException();
    }
    Scalar quickCostFromRoot = quickTrajectoryPlannerContainer.getTrajectoryPlanner().getBest().get().costFromRoot();
    DebugUtils.heuristicConsistencyCheck(quickTrajectoryPlannerContainer.getTrajectoryPlanner());
    System.out.println("***QUICK PLANNER FINISHED***");
    // -- SLOWPLANNER
    partitionScale = Tensors.vector(100, 100);
    RationalScalar resolution = (RationalScalar) RationalScalar.of(12, 1);
    TrajectoryPlannerContainer slowTrajectoryPlannerContainer = DeltaHelper.createGlcAny(RealScalar.of(-0.02), resolution, partitionScale);
    // -- GOALMANAGER
    Iterator<StateTime> iterator = quickTrajectory.iterator();
    List<Region> goalRegions = new ArrayList<>();
    List<Region> goalCheckHelpRegions = new ArrayList<>();
    Tensor radius = Tensors.vector(0.1, 0.1);
    Tensor maxChangePerIterstep = Tensors.vector(1, 1).multiply(slowTrajectoryPlannerContainer.getParameters().getExpandTime()
        .multiply(((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxPossibleChange()));
    Tensor GoalCheckHelpRadius = radius.add(maxChangePerIterstep);
    while (iterator.hasNext()) {
      StateTime next = iterator.next();
      goalRegions.add(new EllipsoidRegion(next.state(), radius));
      goalCheckHelpRegions.add(new EllipsoidRegion(next.state(), GoalCheckHelpRadius));
    }
    DeltaTrajectoryGoalManager trajectoryGoalManager = new DeltaTrajectoryGoalManager(goalRegions, quickTrajectory, radius, //
        ((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxPossibleChange());
    ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(trajectoryGoalManager);
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(3);
    // -- ANYTIMELOOP
    boolean finalGoalFound = false;
    while (!finalGoalFound) {
      List<StateTime> trajectory = new ArrayList<>();
      Optional<GlcNode> finalGoalNode = null;
      // -- ROOTCHANGE
      Stopwatch stopwatch = Stopwatch.started();
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      System.out.println("trajectorys size: " + trajectory.size());
      if (trajectory.size() > 7) {
        //
        StateTime newRootState = trajectory.get(trajectory.size() > 7 ? 1 : 0);
        int increment = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).switchRootToState(newRootState.state());
        slowTrajectoryPlannerContainer.getParameters().increaseDepthLimit(increment);
      }
      stopwatch.stop();
      System.out.println("Rootchange took: " + stopwatch.display_seconds() + "s");
      // // -- GOALCHANGE
      // // Goalchange here is not needed, as getFurthest Goal deasl with it,
      // tic = System.nanoTime();
      // ticTemp = tic;
      // // get GlcNode (€ Goal) with highest Cost (furthest down the path)
      // Optional<StateTime> furthestState = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer//
      // .getTrajectoryPlanner()).getFurthestGoalState();
      // if (furthestState.isPresent()) {
      // if (trajectoryGoalManager.getGoalRegionList().get(trajectoryGoalManager.getGoalRegionList().size() - 1).isMember(furthestState.get().state())) {
      // System.out.println("***Last Goal was found***");
      // finalGoalFound = true;
      // }
      // }
      // Scalar maxSpeed = ((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxPossibleChange();
      // trajectoryGoalManager = new DeltaTrajectoryGoalManager(trajectoryGoalManager.deleteRegionsBefore(furthestState) //
      // , quickTrajectory, radius, maxSpeed);
      // ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(//
      // trajectoryGoalManager, RegionUnion.wrap(goalCheckHelpRegions));
      // if (trajectoryGoalManager.getGoalRegionList().size() < 2)
      // System.err.println("GoalRegion in singular --> FINAL GOAL");
      // tocTemp = System.nanoTime();
      // System.out.println("Goalchange took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      // // --
      // -- EXPANDING
      stopwatch.resetToZero();
      stopwatch.start();
      int expandIter = GlcExpand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), planningTime,
          slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      // int expandIter = GlcExpand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), //
      // planningTime, slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      @SuppressWarnings("unused")
      Optional<StateTime> furthestState = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).getFurthestGoalState();
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      stopwatch.stop();
      System.out.println("Expanding " + expandIter + " Nodes took: " + stopwatch.display_seconds() + "s");
      stopwatchTotal.stop();
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      System.out.println(stopwatchTotal.display_seconds() + " Seconds needed to replan");
      System.out.println("After goal switch needed " + expandIter + " iterations");
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      System.out.println("*****Finished*****");
      DebugUtils.heuristicConsistencyCheck(slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      if (!owlyFrame.jFrame.isVisible() || expandIter < 1)
        break;
      Thread.sleep(10000);
      Scalar slowCostFromRoot = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode().get().costFromRoot();
      System.out.println("Quick planner Cost: " + quickCostFromRoot);
      System.out.println("Slow planner Cost : " + slowCostFromRoot);
    }
    System.out.println("Finished LOOP");
  }
}
