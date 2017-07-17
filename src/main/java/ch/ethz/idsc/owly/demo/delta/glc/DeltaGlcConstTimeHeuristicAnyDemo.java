// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.glc.adapter.TrajectoryPlannerContainer;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

enum DeltaGlcConstTimeHeuristicAnyDemo {
  ;
  public static void main(String[] args) throws Exception {
    // -- Quick Planner init
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(11, 1);
    TrajectoryPlannerContainer quickTrajectoryPlannerContainer = DeltaHelper.createGlc(RealScalar.of(-0.25), quickResolution);
    OwlyFrame quickOwlyFrame = Gui.start();
    quickOwlyFrame.configCoordinateOffset(33, 416);
    quickOwlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Expand.maxDepth(quickTrajectoryPlannerContainer.getTrajectoryPlanner(), DoubleScalar.POSITIVE_INFINITY.number().intValue());
    quickOwlyFrame.setGlc(quickTrajectoryPlannerContainer.getTrajectoryPlanner());
    Optional<GlcNode> optional = quickTrajectoryPlannerContainer.getTrajectoryPlanner().getBest();
    List<StateTime> quickTrajectory = null;
    if (optional.isPresent()) {
      quickTrajectory = GlcNodes.getPathFromRootTo(optional.get());
      Trajectories.print(quickTrajectory);
    } else {
      throw new RuntimeException();
    }
    // s --SLOWPLANNER
    RationalScalar resolution = (RationalScalar) RationalScalar.of(11, 1);
    TrajectoryPlannerContainer slowTrajectoryPlannerContainer = DeltaHelper.createGlcAny(RealScalar.of(-0.25), resolution);
    // GOALMANAGER
    // TODO: needs to be removed from main
    Iterator<StateTime> iterator = quickTrajectory.iterator();
    List<Tensor> quickPath = new ArrayList<>();
    while (iterator.hasNext())
      quickPath.add(iterator.next().x());
    {
      // TODO JONAS build list of regions
      List<Region> regions = new ArrayList<>();
      for (int c = 0; c < 10; ++c) { // !!! use iterator instead of c and "10"
        // regions.add(new EllipsoidRegion(center, radius));
      }
      // TODO JONAS form union of list of regions
      Region union = RegionUnion.wrap(regions);
      // TODO JONAS create and then later use StateTimeRegion
      new TimeInvariantRegion(union);
    }
    // DeltaTrajectoryGoalManager trajectoryGoalManager = new DeltaTrajectoryGoalManager(//
    // quickPath, Tensors.vector(.3, .3), maxSpeed);
    // slowTrajectoryPlanner.changeToGoal(trajectoryGoalManager);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(1);
    while (owlyFrame.jFrame.isVisible()) {
      int expandIter = Expand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), //
          planningTime, slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      if (expandIter < 1)
        break;
      Thread.sleep(1);
    }
  }
}
