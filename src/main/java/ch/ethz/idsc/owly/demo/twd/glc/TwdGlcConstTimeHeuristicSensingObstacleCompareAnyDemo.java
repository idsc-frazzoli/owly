// code by jl
package ch.ethz.idsc.owly.demo.twd.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.twd.TwdControls;
import ch.ethz.idsc.owly.demo.twd.TwdMinCurvatureGoalManager;
import ch.ethz.idsc.owly.demo.twd.TwdMinTimeGoalManager;
import ch.ethz.idsc.owly.demo.twd.TwdStateSpaceModel;
import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.RunCompare;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
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
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
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

enum TwdGlcConstTimeHeuristicSensingObstacleCompareAnyDemo {
  ;
  private static void _run(Scalar resolution, GoalInterface twdGoal, boolean rechabilityRegion) throws Exception {
    boolean heuristic = HeuristicQ.of(twdGoal);
    System.out.println("RUN R=" + resolution + (heuristic ? "H" : "noH") //
        + (rechabilityRegion ? " with Reachability" : " no Reachability"));
    Scalar timeScale = RealScalar.of(2);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(9, 9, 12);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new TwdParameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    parameters.printResolution();
    System.out.println("DomainSize: 1/Eta: " + parameters.getEta().map(n -> RealScalar.ONE.divide(n)));
    TwdStateSpaceModel.createDefault();
    Collection<Flow> controls = TwdControls.createControls(TwdStateSpaceModel.createDefault(), parameters.getResolutionInt());
    // Creating Goals
    Tensor startState = Tensors.vector(0, 0, 0);
    Region environmentRegion = new R2NoiseRegion(RealScalar.of(0.1));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(//
            EuclideanDistanceDiscoverRegion.of(environmentRegion, startState, RealScalar.of(4))));
    // TODO change back to AnyPlannerInterface
    AnyPlannerInterface anyTrajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, twdGoal);
    anyTrajectoryPlanner.switchRootToState(startState);
    GlcExpand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
    // --
    Optional<GlcNode> finalGoalNode = anyTrajectoryPlanner.getFinalGoalNode();
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
    StateTimeTrajectories.print(trajectory);
    boolean useGui = false;
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    if (useGui)
      owlyFrame.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
    RunCompare timingDatabase = new RunCompare(2);
    // -- Anytime loop
    for (int i = 0; i < 13; i++) {
      // while (!finalGoalFound) {
      Thread.sleep(1);
      // ANY
      // -- ROOTCHANGE
      finalGoalNode = anyTrajectoryPlanner.getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      System.out.println("trajectorys size: " + trajectory.size());
      StateTime newRootState = null;
      TrajectoryRegionQuery newObstacleQuery = //
          new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(//
              EuclideanDistanceDiscoverRegion.of(environmentRegion, trajectory.get(0).state(), RealScalar.of(6))));
      timingDatabase.startStopwatchFor(1);
      if (trajectory.size() > 5) {
        //
        newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
        int increment = anyTrajectoryPlanner.switchRootToState(newRootState.state());
        parameters.increaseDepthLimit(increment);
      }
      // -- OBSTACLE CHANGE
      if (rechabilityRegion)
        anyTrajectoryPlanner.obstacleUpdate(newObstacleQuery, new SphericalRegion(trajectory.get(0).state(), RealScalar.of(6).add(RealScalar.ONE)));
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
      // DEFAULT:
      timingDatabase.startStopwatchFor(0);
      int itersStandard = 0;
      StandardTrajectoryPlanner standardTrajectoryPlanner = null;
      if (newRootState != null) {
        standardTrajectoryPlanner = new StandardTrajectoryPlanner(parameters.getEta(), //
            stateIntegrator, controls, newObstacleQuery, twdGoal);
        standardTrajectoryPlanner.insertRoot(newRootState);
        itersStandard = GlcExpand.maxDepth(standardTrajectoryPlanner, parameters.getDepthLimit());
      }
      timingDatabase.stopStopwatchFor(0);
      timingDatabase.saveIterations(itersStandard, 0);
      if (itersStandard < 1)
        break;
      timingDatabase.saveCost(((AbstractAnyTrajectoryPlanner) anyTrajectoryPlanner).getTrajectoryCost(), 1);
      Scalar staCost = DoubleScalar.POSITIVE_INFINITY;
      if (standardTrajectoryPlanner != null)
        staCost = standardTrajectoryPlanner.getBest().get().costFromRoot();
      timingDatabase.saveCost(staCost, 0);
      System.out.println("*****Finished*****");
      timingDatabase.write2lines();
      DebugUtils.heuristicConsistencyCheck((TrajectoryPlanner) anyTrajectoryPlanner);
      if (!owlyFrame.jFrame.isVisible() || itersAny < 1)
        break;
    }
    timingDatabase.write2File("TWDRes" + resolution + (heuristic ? "H" : "noH") + (rechabilityRegion ? "R" : "noR"));
    System.out.println("Finished LOOP");
    owlyFrame.close();
  }

  public static void main(String[] args) throws Exception {
    GoalInterface[] values = new GoalInterface[] { new TwdMinCurvatureGoalManager(Tensors.vector(13, 13, 0), //
        RealScalar.of(0.3), RealScalar.of(1)).getGoalInterface() //
        , new TwdMinTimeGoalManager(Tensors.vector(13, 13, 0), RealScalar.of(0.3), RealScalar.of(1)).getGoalInterface() //
    };
    for (GoalInterface twdGoal : values) {
      _run(RealScalar.of(9), twdGoal, false);
      _run(RealScalar.of(11), twdGoal, false);
      _run(RealScalar.of(13), twdGoal, false);
      _run(RealScalar.of(12), twdGoal, true);
    }
  }
}
