// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.delta.DeltaGoalManagerExt;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaTrajectoryGoalManager;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryPlannerContainer;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
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
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(12, 1);
    TrajectoryPlannerContainer quickTrajectoryPlannerContainer = DeltaHelper.createGlc(RealScalar.of(-0.25), quickResolution);
    Expand.maxDepth(quickTrajectoryPlannerContainer.getTrajectoryPlanner(), DoubleScalar.POSITIVE_INFINITY.number().intValue());
    OwlyFrame quickOwlyFrame = Gui.start();
    quickOwlyFrame.configCoordinateOffset(33, 416);
    quickOwlyFrame.jFrame.setBounds(100, 100, 620, 475);
    quickOwlyFrame.setGlc(quickTrajectoryPlannerContainer.getTrajectoryPlanner());
    Optional<GlcNode> optional = quickTrajectoryPlannerContainer.getTrajectoryPlanner().getBest();
    List<StateTime> quickTrajectory = null;
    if (optional.isPresent()) {
      quickTrajectory = GlcNodes.getPathFromRootTo(optional.get());
      Trajectories.print(quickTrajectory);
    } else {
      throw new RuntimeException();
    }
    System.out.println("***QUICK PLANNER FINISHED***");
    // -- SLOWPLANNER
    RationalScalar resolution = (RationalScalar) RationalScalar.of(12, 1);
    TrajectoryPlannerContainer slowTrajectoryPlannerContainer = DeltaHelper.createGlcAny(RealScalar.of(-0.25), resolution);
    // -- GOALMANAGER
    // TODO: needs to be removed from main
    Iterator<StateTime> iterator = quickTrajectory.iterator();
    List<Region> goalRegions = new ArrayList<>();
    Tensor radius = Tensors.vector(0.1, 0.1);
    while (iterator.hasNext())
      goalRegions.add(new EllipsoidRegion(iterator.next().state(), radius));
    Tensor heuristicCenter = Tensors.vector(2.1, 0.3);
    DeltaTrajectoryGoalManager trajectoryGoalManager = new DeltaTrajectoryGoalManager(goalRegions, quickTrajectory, radius, //
        ((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxInput());
    ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(trajectoryGoalManager);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(1);
    // -- ANYTIMELOOP
    boolean finalGoalFound = false;
    while (!finalGoalFound) {
      List<StateTime> trajectory = null;
      Optional<GlcNode> finalGoalNode = null;
      // --
      // -- GOALCHANGE
      long tic = System.nanoTime();
      long ticTemp = tic;
      // get GlcNode (€ Goal) with highest Cost (furthest down the path)
      Optional<StateTime> furthestState = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer//
          .getTrajectoryPlanner()).getFurthestGoalState();
      int deleteIndex = -1;
      if (furthestState.isPresent()) {
        int index = goalRegions.size();
        while (index > 0) {
          index--;
          if (goalRegions.get(index).isMember(furthestState.get().state())) {
            deleteIndex = index;
            break;
          }
        }
      }
      final int deleteUntilIndex = deleteIndex; // index of first GoalArea which was found
      if (deleteIndex < 0)
        System.out.println("No new Goal was found in last run");
      boolean removed = goalRegions.removeIf(gr -> goalRegions.indexOf(gr) < deleteUntilIndex); // Do not delete Latest (furthest Goal)
      // TODO check if works
      if (removed)
        System.out.println("All Regionparts before/with index: " + deleteUntilIndex + " were removed");// Deleting all goals before the first not found
      System.out.println("size of goal regions list: " + goalRegions.size());
      // TODO: Smart new heuristiccenter:
      // Heuristic Center at next GoalRegion, if found expanding around it
      Scalar maxSpeed = ((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxInput();
      if (!finalGoalFound) {
        trajectoryGoalManager = new DeltaTrajectoryGoalManager(goalRegions, quickTrajectory, radius, maxSpeed);
        ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(//
            trajectoryGoalManager);
      } else {
        if (goalRegions.size() != 1) // only the last Goal is left in the list
          throw new RuntimeException(); // should only include last Goalregion, therefore size ==1
        if (slowTrajectoryPlannerContainer.getTrajectoryPlanner().getGoalQuery() instanceof TrajectoryGoalManager) {
          // only to change GoalManager to final Simple#
          DeltaGoalManagerExt deltaGoalFinal = new DeltaGoalManagerExt(goalRegions.get(0), Trajectories.getLast(quickTrajectory).state(), radius, maxSpeed);
          ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(deltaGoalFinal);
          System.err.println("Changed Goal for last Time");
        }
      }
      long tocTemp = System.nanoTime();
      System.out.println("Goalchange took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      // --
      // -- ROOTCHANGE
      ticTemp = System.nanoTime();
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
      tocTemp = System.nanoTime();
      System.out.println("Rootchange took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      // -- EXPANDING
      ticTemp = System.nanoTime();
      int expandIter = Expand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), //
          planningTime, slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      furthestState = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).getFurthestGoalState();
      // check if furthest Goal is already in last Region in List
      if (furthestState.isPresent()) {
        if (goalRegions.get(goalRegions.size() - 1).isMember(furthestState.get().state())) {
          System.out.println("***Last Goal was found***");
          finalGoalFound = true;
        }
      }
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      tocTemp = System.nanoTime();
      System.out.println("Expanding took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      long toc = System.nanoTime();
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After goal switch needed " + expandIter + " iterations");
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      System.out.println("*****Finished*****");
      if (!owlyFrame.jFrame.isVisible() || expandIter < 1)
        break;
      Thread.sleep(1);
    }
    System.out.println("Finished LOOP");
  }
}
