// code by jl
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import ch.ethz.idsc.owl.data.Stopwatch;
import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.DebugUtils;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.RungeKutta45Integrator;
import ch.ethz.idsc.owl.math.map.BijectionFamily;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.ImageRegion;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.RegionIntersection;
import ch.ethz.idsc.owl.math.region.RegionUnion;
import ch.ethz.idsc.owl.math.region.SphericalRegion;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.delta.DeltaAltStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.DeltaTrajectoryGoalManager;
import ch.ethz.idsc.owly.demo.util.RegionRenders;
import ch.ethz.idsc.owly.demo.util.RunCompare;
import ch.ethz.idsc.owly.demo.util.TrajectoryTranslationFamily;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ResourceData;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.red.Mean;

enum DeltaGlcConstTimeHeuristicAnyDemoMovingObstacles {
  ;
  @SuppressWarnings("unused")
  public static void main(String[] args) throws Exception {
    // -- Quick Planner init
    RationalScalar quickResolution = (RationalScalar) RationalScalar.of(9, 1);
    boolean useGui = true;
    Stopwatch quickPlannerStopwatch = Stopwatch.started();
    // Tensor partitionScale = Tensors.vector(70, 70);
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
    System.out.println("Quickplanner took: " + quickPlannerStopwatch.display_seconds());
    System.out.println("***QUICK PLANNER FINISHED***");
    // -- SLOWPLANNER
    partitionScale = Tensors.vector(120, 120);
    RationalScalar resolution = (RationalScalar) RationalScalar.of(12, 1);
    TrajectoryPlannerContainer slowTrajectoryPlannerContainer = DeltaHelper.createGlcAny(RealScalar.of(-0.02), resolution, partitionScale);
    // -- GOALMANAGER
    Iterator<StateTime> iterator = quickTrajectory.iterator();
    List<Region<Tensor>> goalRegions = new ArrayList<>();
    Tensor radius = Tensors.vector(0.1, 0.1);
    System.out.println("Expandtime: " + slowTrajectoryPlannerContainer.getParameters().getExpandTime());
    // --Trajectory of goals
    // TODO in function/class
    while (iterator.hasNext()) {
      StateTime next = iterator.next();
      goalRegions.add(new EllipsoidRegion(next.state(), radius));
    }
    DeltaTrajectoryGoalManager trajectoryGoalManager = new DeltaTrajectoryGoalManager(goalRegions, quickTrajectory, radius, //
        ((DeltaAltStateSpaceModel) slowTrajectoryPlannerContainer.getStateSpaceModel()).getMaxPossibleChange());
    ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).changeToGoal(trajectoryGoalManager);
    // GUI
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(33, 416);
    owlyFrame.jFrame.setBounds(100, 100, 620, 475);
    // Timings
    Scalar planningTime = RealScalar.of(3.5); // good: 5s
    RunCompare timingDatabase = new RunCompare(1);
    // Obstacles
    Tensor range = Tensors.vector(9, 6.5);
    Tensor obstacleImage = ResourceData.of("/io/delta_free.png");
    Region<Tensor> imageRegion = new ImageRegion(obstacleImage, range, true);
    owlyFrame.addBackground(RegionRenders.create(imageRegion));
    quickOwlyFrame.addBackground(RegionRenders.create(imageRegion));
    quickOwlyFrame.setGlc(quickTrajectoryPlannerContainer.getTrajectoryPlanner());
    Scalar sensingRadius = RealScalar.of(5);
    Supplier<Scalar> supplier = () -> timingDatabase.currentRuntimes.Get(0);
    Flow obstacleFlow = StateSpaceModels.createFlow(quickTrajectoryPlannerContainer.getStateSpaceModel(), //
        Tensors.vectorDouble(0, 0));
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 10), 120 * 10);
    // all the starting positions of the floating obstacles
    List<Tensor> originList = Arrays.asList(//
        Tensors.vector(0.62, 0.35), // good
        Tensors.vector(4, 4.5), // good
        Tensors.vector(1.5, 1), // good
        Tensors.vector(5, 3.0) // good
    );
    List<BijectionFamily> obstacleTrajectoriesList = new ArrayList<>();
    for (Tensor entry : originList)
      obstacleTrajectoriesList.add(TrajectoryTranslationFamily.create(stateIntegrator, new StateTime(entry, RealScalar.ZERO), obstacleFlow));
    // all the radius of the floating obstacles
    List<Scalar> radiusList = Arrays.asList(//
        RealScalar.of(0.2), //
        RealScalar.of(0.6), //
        RealScalar.of(0.4), //
        RealScalar.of(0.5) //
    );
    if (radiusList.size() != originList.size())
      throw new RuntimeException();
    List<Region<StateTime>> floatingObstaclesList = new ArrayList<>();
    for (int i = 0; i < radiusList.size(); i++) {
      TensorUnaryOperator fwd = obstacleTrajectoriesList.get(i).forward(supplier.get());
      floatingObstaclesList.add(new TimeInvariantRegion(new SphericalRegion(fwd.apply(Tensors.vector(0, 0)), radiusList.get(i))));
    }
    // -- ANYTIMELOOP
    boolean finalGoalFound = false;
    int iter = 0;
    Tensor trajectorySizeSum = Tensors.vector();
    Tensor listUpdateTime = Tensors.vector();
    while (!finalGoalFound && iter < 300) {
      List<StateTime> trajectory = new ArrayList<>();
      Optional<GlcNode> finalGoalNode = null;
      // -- ROOTCHANGE
      Stopwatch stopwatch = Stopwatch.started();
      timingDatabase.startStopwatchFor(0);
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      else {
        Optional<GlcNode> bestEstimate = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).getBestOrElsePeek();
        if (bestEstimate.isPresent())
          trajectory = GlcNodes.getPathFromRootTo(bestEstimate.get());
      }
      System.out.println("trajectorys size: " + trajectory.size());
      Scalar currentTime = supplier.get();
      if (iter >= 1) { // only move forward after initial expansion
        trajectorySizeSum.append(RealScalar.of(trajectory.size()));
        boolean test = trajectory.removeIf(st -> Scalars.lessThan(st.time(), currentTime.add(planningTime)));
        // only nodes in the future are kept
        if (!test)
          System.out.println("Did not move forward a node");
      }
      if (trajectory.size() >= 1) {
        StateTime newRootState = trajectory.get(0);
        int increment = ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).switchRootToState(newRootState);
        slowTrajectoryPlannerContainer.getParameters().increaseDepthLimit(increment);
      } else if (trajectory.size() == 0 && iter != 0) {
        System.err.println("Too slow expansion of tree");
        break;
      }
      stopwatch.stop();
      // -- OBSTACLE UPDATE
      if (trajectory.size() >= 1) {
        // Definition of the floating Obstacles
        Region<StateTime> floatingObstacles = RegionUnion.wrap(//
            createFloatingObstacles(obstacleTrajectoriesList, radiusList, supplier));
        Region<StateTime> discoveredFloatingObstacle = RegionIntersection.wrap(Arrays.asList( //
            new TimeInvariantRegion(new SphericalRegion(trajectory.get(0).state(), sensingRadius)), floatingObstacles));
        // Combination of the discovered new obstacles and the map
        stopwatch.start();
        TrajectoryRegionQuery newObstacle = new SimpleTrajectoryRegionQuery( //
            RegionUnion.wrap(Arrays.asList(new TimeInvariantRegion(imageRegion), discoveredFloatingObstacle)));
        ((OptimalAnyTrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner()).obstacleUpdate(newObstacle);
        System.out.println("Obstaclechange took: " + stopwatch.display_seconds() + "s");
        // listUpdateTime.append(RealScalar.of(stopwatch.display_seconds()));
      }
      // stopwatch.stop();
      // stopwatch.resetToZero();
      // -- EXPANDING
      // stopwatch.start();
      int expandIter = 0;
      expandIter = GlcExpand.constTime(slowTrajectoryPlannerContainer.getTrajectoryPlanner(), planningTime,
          slowTrajectoryPlannerContainer.getParameters().getDepthLimit());
      iter++; // One additional expansion was conducted
      finalGoalNode = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode();
      stopwatch.stop();
      timingDatabase.pauseStopwatchFor(0);
      timingDatabase.saveIterations(expandIter, 0);
      System.out.println("Expanding " + expandIter + " Nodes took: " + stopwatch.display_seconds() + "s");
      listUpdateTime.append(RealScalar.of(stopwatch.display_seconds()));
      if (useGui) {
        owlyFrame.setGlc((TrajectoryPlanner) slowTrajectoryPlannerContainer.getTrajectoryPlanner());
        owlyFrame.addBackground(RegionRenders.create(imageRegion));
      }
      List<StateTime> Trajectory = null;
      if (optional.isPresent()) {
        Trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
        StateTimeTrajectories.print(Trajectory);
      } else {
        throw new RuntimeException();
      }
      System.out.println("*****Finished*****");
      DebugUtils.heuristicConsistencyCheck(slowTrajectoryPlannerContainer.getTrajectoryPlanner());
      if (!owlyFrame.jFrame.isVisible() || expandIter < 1)
        break;
      Scalar slowCostFromRoot = slowTrajectoryPlannerContainer.getTrajectoryPlanner().getFinalGoalNode().get().costFromRoot();
      timingDatabase.saveCost(slowCostFromRoot, 0);
      timingDatabase.printcurrent();
      timingDatabase.write2lines();
      Thread.sleep(1000);
    }
    boolean test = HeuristicQ.of(trajectoryGoalManager);
    String filename = "GLCR" + resolution + (test ? "H" : "noH");
    timingDatabase.write2File(filename);
    System.out.println("Finished LOOP");
    Tensor meanTrajectorySize = Mean.of(trajectorySizeSum);
    Tensor meanUpdateTime = Mean.of(listUpdateTime);
    System.out.println("Average node length of trajectory: " + meanTrajectorySize);
    System.out.println("With a timelength of " + meanTrajectorySize.multiply(slowTrajectoryPlannerContainer.getParameters().getExpandTime()) + "s");
    System.out.println("Average Expansiontime: " + meanUpdateTime + "s == f: " + meanUpdateTime.map(Scalar::reciprocal));
  }

  private static List<Region<StateTime>> createFloatingObstacles(List<BijectionFamily> obstacleTrajectoriesList, List<Scalar> radiusList,
      Supplier<Scalar> supplier) {
    List<Region<StateTime>> floatingObstaclesList = new ArrayList<>();
    for (int i = 0; i < radiusList.size(); i++) {
      TensorUnaryOperator fwd = obstacleTrajectoriesList.get(i).forward(supplier.get());
      floatingObstaclesList.add(new TimeInvariantRegion(new SphericalRegion(fwd.apply(Tensors.vector(0, 0)), radiusList.get(i))));
    }
    return floatingObstaclesList;
  }
}
