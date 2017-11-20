// code by jl
package ch.ethz.idsc.owly.demo.se2.twd;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.demo.util.RunCompare;
import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
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
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum TwdGlcHeuristicSensingObstacleCompareAnyDemo {
  ;
  public static final TwdDuckieFlows TWDCONFIG = new TwdDuckieFlows(RealScalar.ONE, RealScalar.ONE);

  private static void _run(Scalar resolution, GoalInterface twdGoal, boolean rechabilityRegion, Scalar sensingRadius) throws Exception {
    boolean heuristic = HeuristicQ.of(twdGoal);
    System.out.println("RUN R=" + resolution + (heuristic ? "H" : "noH") //
        + (rechabilityRegion ? " with Reachability" : " no Reachability"));
    Scalar timeScale = RealScalar.of(8);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(5, 5, 10);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new TwdParameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    parameters.printResolution();
    System.out.println("DomainSize: 1/Eta: " + parameters.getEta().map(n -> RealScalar.ONE.divide(n)));
    Collection<Flow> controls = TWDCONFIG.getFlows(parameters.getResolutionInt());
    // Creating Goals
    Tensor startState = Tensors.vector(0, 0, 0);
    Region<Tensor> environmentRegion = new R2NoiseRegion(RealScalar.of(0.1));
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        EuclideanDistanceDiscoverRegion.of(environmentRegion, startState, RealScalar.of(4)));
    AnyPlannerInterface anyTrajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, twdGoal);
    anyTrajectoryPlanner.switchRootToState(startState);
    GlcExpand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
    // --
    Optional<GlcNode> finalGoalNode = anyTrajectoryPlanner.getFinalGoalNode();
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
    StateTimeTrajectories.print(trajectory);
    boolean useGui = false;
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    if (useGui)
      owlyFrame.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
    RunCompare timingDatabase = new RunCompare(2);
    // -- Anytime loop
    for (int i = 0; i < 18; i++) {
      // while (!finalGoalFound) {
      Thread.sleep(1);
      // ANY
      // -- ROOTCHANGE
      finalGoalNode = anyTrajectoryPlanner.getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      System.out.println("trajectorys size: " + trajectory.size());
      StateTime newRootState = null;
      TrajectoryRegionQuery newObstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
          EuclideanDistanceDiscoverRegion.of(environmentRegion, trajectory.get(0).state(), sensingRadius));
      timingDatabase.startStopwatchFor(1);
      if (trajectory.size() > 5) {
        //
        newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
        int increment = anyTrajectoryPlanner.switchRootToState(newRootState.state());
        parameters.increaseDepthLimit(increment);
      }
      // -- OBSTACLE CHANGE
      if (rechabilityRegion)
        anyTrajectoryPlanner.obstacleUpdate(newObstacleQuery, new SphericalRegion(trajectory.get(0).state(), sensingRadius.add(RealScalar.ONE)));
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
      timingDatabase.printcurrent();
      ((OptimalAnyTrajectoryPlanner) anyTrajectoryPlanner).printTimes();
      standardTrajectoryPlanner.printTimes();
      DebugUtils.heuristicConsistencyCheck((TrajectoryPlanner) anyTrajectoryPlanner);
      if (!owlyFrame.jFrame.isVisible() || itersAny < 1)
        break;
    }
    timingDatabase.write2File("TWDRes" + resolution + (heuristic ? "H" : "noH") + (rechabilityRegion ? "R" : "noR") + "S" + sensingRadius);
    System.out.println("Finished LOOP");
    // owlyFrame.close();
  }

  public static void main(String[] args) throws Exception {
    GoalInterface[] values = new GoalInterface[] { //
        // new TwdMinCurvatureGoalManager(Tensors.vector(13, 13, 0), RealScalar.of(0.3), RealScalar.of(1)).getGoalInterface()
        Se2MinTimeGoalManager.create(Tensors.vector(13, 13, 0), Tensors.vector(0.3, 0.3, 1), //
            // let's hope the controls lead to the right results
            TWDCONFIG.getFlows(8)) //
        // new TwdMinTimeGoalManager(Tensors.vector(13, 13, 0), RealScalar.of(0.3), RealScalar.of(1)).getGoalInterface()
        // new TwdNoHeuristicGoalManager(Tensors.vector(6, 6, 0), Tensors.vector(0.3, 0.3, 1)).getGoalInterface() //
    };
    for (GoalInterface twdGoal : values) {
      _run(RealScalar.of(10), twdGoal, false, RealScalar.of(6));
      // _run(RealScalar.of(9), twdGoal, false, RealScalar.of(8));
      // _run(RealScalar.of(11), twdGoal, false);
      // _run(RealScalar.of(9), twdGoal, true);
      // _run(RealScalar.of(11), twdGoal, true);
    }
  }
}
