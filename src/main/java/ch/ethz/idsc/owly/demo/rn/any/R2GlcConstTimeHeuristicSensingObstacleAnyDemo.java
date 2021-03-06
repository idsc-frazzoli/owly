// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.GlcNodes;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.any.AnyPlannerInterface;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.DebugUtils;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.flow.EulerIntegrator;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.region.SphericalRegion;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2Flows;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2GlcConstTimeHeuristicSensingObstacleAnyDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(2.5);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(20, 20);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar runTime = RealScalar.of(0.3);
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    parameters.printResolution();
    System.out.println("DomainSize: 1/Eta: " + parameters.getEta().map(n -> RealScalar.ONE.divide(n)));
    R2Flows r2Config = new R2Flows(RealScalar.ONE);
    Collection<Flow> controls = r2Config.getFlows(parameters.getResolutionInt());
    // Creating Goals
    Tensor startState = Tensors.vector(-3, 0);
    GoalInterface rnGoal = RnMinDistSphericalGoalManager.create(Tensors.vector(10, -4), RealScalar.of(0.3));
    Region<Tensor> environmentRegion = new R2NoiseRegion(RealScalar.of(0.1));
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        EuclideanDistanceDiscoverRegion.of(environmentRegion, startState, RealScalar.of(4)));
    AnyPlannerInterface anyPlannerInterface = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    anyPlannerInterface.switchRootToState(new StateTime(Tensors.vector(-3, 0), RealScalar.ZERO));
    GlcExpand.constTime(anyPlannerInterface, runTime, parameters.getDepthLimit());
    // --
    Optional<GlcNode> finalGoalNode = anyPlannerInterface.getFinalGoalNode();
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
    StateTimeTrajectories.print(trajectory);
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
    // -- Anytime loop
    for (int i = 0; i < 10; i++) {
      // while (!finalGoalFound) {
      Thread.sleep(1);
      long tic = System.nanoTime();
      // -- ROOTCHANGE
      finalGoalNode = anyPlannerInterface.getFinalGoalNode();
      if (finalGoalNode.isPresent())
        trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      System.out.println("trajectorys size: " + trajectory.size());
      if (trajectory.size() > 5) {
        //
        StateTime newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
        int increment = anyPlannerInterface.switchRootToState(newRootState);
        parameters.increaseDepthLimit(increment);
      }
      // -- OBSTACLE CHANGE
      TrajectoryRegionQuery newObstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
          EuclideanDistanceDiscoverRegion.of(environmentRegion, trajectory.get(0).state(), RealScalar.of(4)));
      anyPlannerInterface.obstacleUpdate(newObstacleQuery, new SphericalRegion(trajectory.get(0).state(), RealScalar.of(4).add(RealScalar.ONE)));
      // // -- GOALCHANGE
      // ticTemp = tic;
      // Optional<StateTime> furthestState = trajectoryPlanner.getFurthestGoalState(rnGoal.getGoalRegionList());
      // if (furthestState.isPresent()) {
      // if (rnGoal.getGoalRegionList().get(rnGoal.getGoalRegionList().size() - 1).isMember(furthestState.get().state())) {
      // System.out.println("***Last Goal was found***");
      // finalGoalFound = true;
      // }
      // }
      // System.out.println("Current size of goal regions list: " + rnGoal.getGoalRegionList().size());
      // // only change goal if we are not at the end yet
      // // creates new RegionUnin form Regionlist and puts Heuristic to next Goal in RegionList
      // rnGoal = new RnTrajectoryGoalManager(rnGoal.deleteRegionsBefore(furthestState), precomputedTrajectory, radius);
      // trajectoryPlanner.changeToGoal(rnGoal);
      // if (rnGoal.getGoalRegionList().size() < 2)
      // System.err.println("changed to single region goal --> Last Change");
      // tocTemp = System.nanoTime();
      // System.out.println("Goalchange took: " + (tocTemp - ticTemp) * 1e-9 + "s");
      // -- EXPANDING
      int expandIter = GlcExpand.constTime(anyPlannerInterface, runTime, parameters.getDepthLimit());
      owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
      // check if furthest Goal is already in last Region in List
      trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      // --
      long toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After goal switch needed " + expandIter + " iterations");
      System.out.println("*****Finished*****");
      DebugUtils.heuristicConsistencyCheck((TrajectoryPlanner) anyPlannerInterface);
      if (!owlyFrame.jFrame.isVisible() || expandIter < 1)
        break;
    }
    System.out.println("Finished LOOP");
  }
}
