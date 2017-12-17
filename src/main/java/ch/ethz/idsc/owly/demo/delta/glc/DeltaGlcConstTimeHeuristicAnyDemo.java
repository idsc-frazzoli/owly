// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.data.Lists;
import ch.ethz.idsc.owl.data.Stopwatch;
import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.DebugUtils;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.delta.DeltaAltStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaTrajectoryGoalManager;
import ch.ethz.idsc.owly.demo.util.RunCompare;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum DeltaGlcConstTimeHeuristicAnyDemo {
  ;
  @SuppressWarnings("unused")
  public static void main(String[] args) throws Exception {
    // -- Quick Planner init
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(10, 1);
    boolean useGui = true;
    Tensor partitionScale = Tensors.vector(120, 120);
    TrajectoryPlannerContainer quickTrajectoryPlannerContainer = DeltaHelper.createGlc(RealScalar.of(-0.02), quickResolution, partitionScale);
    GlcExpand.maxDepth(quickTrajectoryPlannerContainer.getTrajectoryPlanner(), DoubleScalar.POSITIVE_INFINITY.number().intValue());
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
    List<Region<Tensor>> goalRegions = new ArrayList<>();
    List<Region<Tensor>> goalCheckHelpRegions = new ArrayList<>();
    Tensor radius = Tensors.vector(0.1, 0.1);
    Tensor maxChangePerIterstep = Tensors.vector(1, 1).multiply(slowTrajectoryPlannerContainer.getParameters().getExpandTime()
        .multiply(((DeltaAltStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxPossibleChange()));
    Tensor GoalCheckHelpRadius = radius.add(maxChangePerIterstep);
    while (iterator.hasNext()) {
      StateTime next = iterator.next();
      goalRegions.add(new EllipsoidRegion(next.state(), radius));
      goalCheckHelpRegions.add(new EllipsoidRegion(next.state(), GoalCheckHelpRadius));
    }
    DeltaTrajectoryGoalManager trajectoryGoalManager = new DeltaTrajectoryGoalManager(goalRegions, quickTrajectory, radius, //
        ((DeltaAltStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxPossibleChange());
    ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(trajectoryGoalManager);
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    Scalar planningTime = RealScalar.of(3);
    RunCompare timingDatabase = new RunCompare(2);
    // -- ANYTIMELOOP
    boolean finalGoalFound = false;
    int iter = 0;
    while (!finalGoalFound && iter < 30) {
      iter++;
      List<StateTime> trajectory = new ArrayList<>();
      Optional<GlcNode> finalGoalNode = null;
      // -- ROOTCHANGE
      Stopwatch stopwatch = Stopwatch.started();
      timingDatabase.startStopwatchFor(1);
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      System.out.println("trajectorys size: " + trajectory.size());
      if (false) {
        // if (trajectory.size() > 7) {
        //
        StateTime newRootState = trajectory.get(trajectory.size() > 7 ? 1 : 0);
        int increment = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).switchRootToState(newRootState);
        slowTrajectoryPlannerContainer.getParameters().increaseDepthLimit(increment);
      }
      stopwatch.stop();
      System.out.println("Rootchange took: " + stopwatch.display_seconds() + "s");
      // -- EXPANDING
      stopwatch.resetToZero();
      stopwatch.start();
      int expandIter = GlcExpand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), planningTime,
          slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      // int expandIter = GlcExpand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), //
      // planningTime, slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      // Optional<StateTime> furthestState = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).getFurthestGoalState();
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      stopwatch.stop();
      timingDatabase.pauseStopwatchFor(1);
      timingDatabase.saveIterations(expandIter, 1);
      System.out.println("Expanding " + expandIter + " Nodes took: " + stopwatch.display_seconds() + "s");
      if (useGui)
        owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      System.out.println("After goal switch needed " + expandIter + " iterations");
      {
        timingDatabase.startStopwatchFor(0);
        Tensor tempGoal = Lists.getLast(trajectory).state();
        TrajectoryPlannerContainer standardTrajectoryPlannerContainer = DeltaHelper.createGlcFromRootToGoal(RealScalar.of(-0.02), resolution, partitionScale,
            trajectory.get(0).state(), tempGoal);
        int interSta = GlcExpand.maxDepth(standardTrajectoryPlannerContainer.getTrajectoryPlanner(), DoubleScalar.POSITIVE_INFINITY.number().intValue());
        timingDatabase.stopStopwatchFor(0);
        timingDatabase.saveIterations(interSta, 0);
        timingDatabase.saveCost(standardTrajectoryPlannerContainer.getTrajectoryPlanner().getBest().get().costFromRoot(), 0);
      }
      System.out.println("*****Finished*****");
      DebugUtils.heuristicConsistencyCheck(slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      if (!owlyFrame.jFrame.isVisible() || expandIter < 1)
        break;
      Scalar slowCostFromRoot = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode().get().costFromRoot();
      timingDatabase.saveCost(slowCostFromRoot, 1);
      timingDatabase.printcurrent();
      timingDatabase.write2lines();
      Thread.sleep(1000);
    }
    boolean test = HeuristicQ.of(trajectoryGoalManager);
    String filename = "GLCR" + resolution + (test ? "H" : "noH");
    timingDatabase.write2File(filename);
    System.out.println("Finished LOOP");
  }
}
