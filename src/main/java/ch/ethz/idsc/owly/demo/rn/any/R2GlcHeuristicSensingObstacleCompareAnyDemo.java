// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.RunCompare;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AbstractAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.DebugUtils;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
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

enum R2GlcHeuristicSensingObstacleCompareAnyDemo {
  ;
  private static void _run(Scalar resolution, GoalInterface rnGoal, boolean rechabilityRegion) throws Exception {
    boolean heuristic = HeuristicQ.of(rnGoal);
    System.out.println("RUN R=" + resolution + (heuristic ? "H" : "noH") //
        + (rechabilityRegion ? " with Reachability" : " no Reachability"));
    Scalar timeScale = RealScalar.of(5);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(20, 20);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Scalar sensingRadius = RealScalar.of(10);
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    parameters.printResolution();
    System.out.println("DomainSize: 1/Eta: " + parameters.getEta().map(n -> RealScalar.ONE.divide(n)));
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    // Creating Goals
    Tensor startState = Tensors.vector(-3, 0);
    Region environmentRegion = new R2NoiseRegion(RealScalar.of(0.1));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(//
            EuclideanDistanceDiscoverRegion.of(environmentRegion, startState, sensingRadius)));
    // TODO change back to AnyPlannerInterface
    AnyPlannerInterface anyTrajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    anyTrajectoryPlanner.switchRootToState(startState);
    GlcExpand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
    ((OptimalAnyTrajectoryPlanner) anyTrajectoryPlanner).printTimes();
    // --
    Optional<GlcNode> finalGoalNode = anyTrajectoryPlanner.getFinalGoalNode();
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
    // StateTimeTrajectories.print(trajectory);
    boolean useGui = false;
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    if (useGui)
      owlyFrame.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
    RunCompare timingDatabase = new RunCompare(2);
    // -- Anytime loop
    for (int i = 0; i < 13; i++) {
      // while (!finalGoalFound) {
      Thread.sleep(1);
      // SETUP
      finalGoalNode = anyTrajectoryPlanner.getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      System.out.println("trajectorys size: " + trajectory.size());
      StateTime newRootState = null;
      newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
      TrajectoryRegionQuery newObstacleQuery = //
          new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(//
              EuclideanDistanceDiscoverRegion.of(environmentRegion, trajectory.get(0).state(), sensingRadius)));
      //
      // DEFAULT:
      StandardTrajectoryPlanner standardTrajectoryPlanner = null;
      {
        timingDatabase.startStopwatchFor(0);
        int itersStandard = 0;
        if (newRootState != null) {
          standardTrajectoryPlanner = new StandardTrajectoryPlanner(parameters.getEta(), //
              stateIntegrator, controls, newObstacleQuery, rnGoal);
          standardTrajectoryPlanner.insertRoot(newRootState);
          itersStandard = GlcExpand.maxDepth(standardTrajectoryPlanner, parameters.getDepthLimit());
        }
        timingDatabase.stopStopwatchFor(0);
        timingDatabase.saveIterations(itersStandard, 0);
        if (itersStandard < 1)
          break;
      }
      {
        timingDatabase.startStopwatchFor(1);
        // ANY
        // -- ROOTCHANGE
        if (trajectory.size() > 2) {
          //
          int increment = anyTrajectoryPlanner.switchRootToState(newRootState.state());
          parameters.increaseDepthLimit(increment);
        }
        // -- OBSTACLE CHANGE
        if (rechabilityRegion)
          anyTrajectoryPlanner.obstacleUpdate(newObstacleQuery, //
              new SphericalRegion(trajectory.get(0).state(), sensingRadius.add(RealScalar.ONE)));
        else
          anyTrajectoryPlanner.obstacleUpdate(newObstacleQuery);
        // -- EXPANDING
        int itersAny = GlcExpand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
        timingDatabase.saveIterations(itersAny, 1);
        // check if furthest Goal is already in last Region in List
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
        // --
        timingDatabase.stopStopwatchFor(1);
        if (useGui)
          owlyFrame.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
      }
      // DATA COLLECTION
      timingDatabase.saveCost(((AbstractAnyTrajectoryPlanner) anyTrajectoryPlanner).getTrajectoryCost(), 1);
      Scalar staCost = DoubleScalar.POSITIVE_INFINITY;
      if (standardTrajectoryPlanner != null)
        staCost = standardTrajectoryPlanner.getBest().get().costFromRoot();
      timingDatabase.saveCost(staCost, 0);
      standardTrajectoryPlanner.printTimes();
      ((OptimalAnyTrajectoryPlanner) anyTrajectoryPlanner).printTimes();
      timingDatabase.printcurrent();
      System.out.println("*****Finished*****");
      timingDatabase.write2lines();
      DebugUtils.heuristicConsistencyCheck((TrajectoryPlanner) anyTrajectoryPlanner);
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
    timingDatabase.write2File("Res" + resolution + (heuristic ? "H" : "noH") + (rechabilityRegion ? "R" : "noR"));
    System.out.println("Finished LOOP");
    owlyFrame.close();
  }

  public static void main(String[] args) throws Exception {
    GoalInterface[] values = new GoalInterface[] { RnMinDistSphericalGoalManager.create(Tensors.vector(25, 25), RealScalar.of(0.3)),
        // new RnSimpleCircleGoalManager(Tensors.vector(20, 20), RealScalar.of(0.3)) //
    };
    for (GoalInterface rnGoal : values) {
      // _run(RealScalar.of(8), rnGoal, true);
      _run(RealScalar.of(20), rnGoal, false);
      // _run(RealScalar.of(12), rnGoal, true);
      // _run(RealScalar.of(12), rnGoal, true);
    }
  }
}
