// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.delta.DeltaGoalManagerExt;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
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
import ch.ethz.idsc.owly.math.region.RegionUnion;
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
    Tensor radius = Tensors.vector(0.05, 0.05);
    while (iterator.hasNext())
      goalRegions.add(new EllipsoidRegion(iterator.next().x(), radius));
    RegionUnion goalRegion = new RegionUnion(goalRegions);
    Tensor heuristicCenter = Tensors.vector(2.1, 0.3);
    DeltaGoalManagerExt trajectoryGoalManager = new DeltaGoalManagerExt(goalRegion, heuristicCenter, //
        ((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxInput());
    ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(trajectoryGoalManager);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(1);
    // -- ANYTIMELOOP
    while (owlyFrame.jFrame.isVisible()) {
      // get GlcNode (€ Goal) with highest Cost (furthest down the path)
      Optional<GlcNode> furthest = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer//
          .getTrajectoryPlanner()).getFurthestGoalNode();
      int deleteIndex = -1;
      if (furthest.isPresent()) {
        // checks if last Region was found
        if (goalRegions.get(goalRegions.size() - 1).isMember(furthest.get().state())) {
          System.out.println("***Last Goal was found***");
          break; // stops Anytimeloop
        }
        int iter = goalRegions.size();
        while (iter > 0) {
          iter--;
          // searches furthest Subregion corresponding to furthest GoalNode
          if (goalRegions.get(iter).isMember(furthest.get().state())) {
            deleteIndex = iter;
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
      Region goalRegionAny = new RegionUnion(goalRegions);
      trajectoryGoalManager = new DeltaGoalManagerExt(goalRegionAny, heuristicCenter, // // TODO: Smart new heuristiccenter
          ((DeltaStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxInput());
      // TODO JONAS: write is regionUnion changing? as contructed)
      ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(//
          trajectoryGoalManager); // TODO JONAS Needed as Region Union is changed?
      //
      int expandIter = Expand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), //
          planningTime, slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      if (expandIter < 1)
        break;
      Thread.sleep(1);
    }
  }
}
