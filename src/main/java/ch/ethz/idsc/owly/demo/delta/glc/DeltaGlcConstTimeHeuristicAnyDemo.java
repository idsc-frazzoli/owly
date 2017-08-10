// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaTrajectoryGoalManager;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryPlannerContainer;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum DeltaGlcConstTimeHeuristicAnyDemo {
  ;
  public static void main(String[] args) throws Exception {
    // -- Quick Planner init
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(10, 1);
    Tensor partitionScale = Tensors.vector(2e26, 2e26);
    TrajectoryPlannerContainer quickTrajectoryPlannerContainer = DeltaHelper.createGlc(RealScalar.of(-0.5), quickResolution, partitionScale);
    Expand.maxDepth(quickTrajectoryPlannerContainer.getTrajectoryPlanner(), DoubleScalar.POSITIVE_INFINITY.number().intValue());
    OwlyFrame quickOwlyFrame = Gui.start();
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
    DebugUtils.heuristicConsistencyCheck(quickTrajectoryPlannerContainer.getTrajectoryPlanner());
    System.out.println("***QUICK PLANNER FINISHED***");
    // -- SLOWPLANNER
    partitionScale = Tensors.vector(6e29, 6e29);
    RationalScalar resolution = (RationalScalar) RationalScalar.of(14, 1);
    TrajectoryPlannerContainer slowTrajectoryPlannerContainer = DeltaHelper.createGlcAny(RealScalar.of(-0.5), resolution, partitionScale);
    // -- GOALMANAGER
    // TODO needs to be removed from main
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
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(3);
    // -- ANYTIMELOOP
    boolean finalGoalFound = false;
    int iteration = 0;
    while (!finalGoalFound) {
      List<StateTime> trajectory = new ArrayList<>();
      Optional<GlcNode> finalGoalNode = null;
      // --
      // -- ROOTCHANGE
      long ticTemp = System.nanoTime();
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      System.out.println("trajectorys size: " + trajectory.size());
      if (trajectory.size() > 5) {
        //
        StateTime newRootState = trajectory.get(trajectory.size() > 7 ? 2 : 0);
        int increment = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).switchRootToState(newRootState.state());
        slowTrajectoryPlannerContainer.getParameters().increaseDepthLimit(increment);
      }
      long tocTemp = System.nanoTime();
      System.out.println("Rootchange took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      // -- GOALCHANGE
      // Goalchange here is not needed, as getFurthest Goal deasl with it,
      long tic = System.nanoTime();
      ticTemp = tic;
      // get GlcNode (€ Goal) with highest Cost (furthest down the path)
      Optional<StateTime> furthestState = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer//
          .getTrajectoryPlanner()).getFurthestGoalState(trajectoryGoalManager.getGoalRegionList());
      if (furthestState.isPresent()) {
        if (trajectoryGoalManager.getGoalRegionList().get(trajectoryGoalManager.getGoalRegionList().size() - 1).isMember(furthestState.get().state())) {
          System.out.println("***Last Goal was found***");
          finalGoalFound = true;
        }
      }
      // TODO Smart new heuristiccenter:
      // Heuristic Center at next GoalRegion, if found expanding around it
      Scalar maxSpeed = ((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxPossibleChange();
      trajectoryGoalManager = new DeltaTrajectoryGoalManager(trajectoryGoalManager.deleteRegionsBefore(furthestState) //
          , quickTrajectory, radius, maxSpeed);
      ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(//
          trajectoryGoalManager, RegionUnion.wrap(goalCheckHelpRegions));
      if (trajectoryGoalManager.getGoalRegionList().size() < 2)
        System.err.println("GoalRegion in singular --> FINAL GOAL");
      tocTemp = System.nanoTime();
      System.out.println("Goalchange took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      // --
      // -- EXPANDING
      ticTemp = System.nanoTime();
      int expandIter = Expand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), //
          planningTime, slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      furthestState = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).getFurthestGoalState(goalRegions);
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      tocTemp = System.nanoTime();
      System.out.println("Expanding " + expandIter + " Nodes took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      iteration++;
      System.out.println("In " + iteration + "st Expandstep");
      long toc = System.nanoTime();
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After goal switch needed " + expandIter + " iterations");
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      System.out.println("*****Finished*****");
      DebugUtils.heuristicConsistencyCheck(slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      if (!owlyFrame.jFrame.isVisible() || expandIter < 1)
        break;
      Thread.sleep(1);
    }
    System.out.println("Finished LOOP");
  }
}
