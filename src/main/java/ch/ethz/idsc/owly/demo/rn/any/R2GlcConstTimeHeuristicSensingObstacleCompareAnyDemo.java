// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.EuclideanDistanceDiscoverRegion;
import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.demo.util.UserHome;
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

enum R2GlcConstTimeHeuristicSensingObstacleCompareAnyDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(15);
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
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    // Creating Goals
    Tensor startState = Tensors.vector(-3, 0);
    GoalInterface rnGoal = RnMinDistSphericalGoalManager.create(Tensors.vector(10, 0), RealScalar.of(0.3));
    Region environmentRegion = new R2NoiseRegion(RealScalar.of(0.1));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(//
            EuclideanDistanceDiscoverRegion.of(environmentRegion, startState, RealScalar.of(4))));
    // TODO change back to AnyPlannerInterface
    AnyPlannerInterface anyTrajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    anyTrajectoryPlanner.switchRootToState(startState);
    GlcExpand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
    // --
    Optional<GlcNode> finalGoalNode = anyTrajectoryPlanner.getFinalGoalNode();
    List<StateTime> trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
    StateTimeTrajectories.print(trajectory);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    owlyFrame.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
    Path path = UserHome.file("R2Comparison.csv").toPath();
    List<String> lines = Arrays.asList("timeStandard, timeDiff, iterationsDiff, CostDiff");
    Files.write(path, lines, Charset.forName("UTF-8"));
    // -- Anytime loop
    for (int i = 0; i < 10; i++) {
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
              EuclideanDistanceDiscoverRegion.of(environmentRegion, trajectory.get(0).state(), RealScalar.of(4))));
      long ticAny = System.nanoTime();
      if (trajectory.size() > 5) {
        //
        newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
        int increment = anyTrajectoryPlanner.switchRootToState(newRootState.state());
        parameters.increaseDepthLimit(increment);
      }
      // -- OBSTACLE CHANGE
      anyTrajectoryPlanner.obstacleUpdate(newObstacleQuery, new SphericalRegion(trajectory.get(0).state(), RealScalar.of(4).add(RealScalar.ONE)));
      // -- EXPANDING
      int itersAny = GlcExpand.maxDepth(anyTrajectoryPlanner, parameters.getDepthLimit());
      // check if furthest Goal is already in last Region in List
      trajectory = GlcNodes.getPathFromRootTo(finalGoalNode.get());
      // --
      long tocAny = System.nanoTime();
      owlyFrame.setGlc((TrajectoryPlanner) anyTrajectoryPlanner);
      // DEFAULT:
      long ticStandard = System.nanoTime();
      int itersStandard = 0;
      StandardTrajectoryPlanner standardTrajectoryPlanner = null;
      if (newRootState != null) {
        standardTrajectoryPlanner = new StandardTrajectoryPlanner(parameters.getEta(), //
            stateIntegrator, controls, newObstacleQuery, rnGoal);
        standardTrajectoryPlanner.insertRoot(newRootState);
        itersStandard = GlcExpand.maxDepth(standardTrajectoryPlanner, parameters.getDepthLimit());
      }
      long tocStandard = System.nanoTime();
      System.out.println("****COMPARING TIME:  ****");
      Scalar anyTimeDiff = RealScalar.of((tocAny - ticAny) * 1e-9);
      Scalar staTimeDiff = RealScalar.of((tocStandard - ticStandard) * 1e-9);
      System.out.println("ANY: " + anyTimeDiff);
      System.out.println("STA: " + staTimeDiff);
      System.out.println("****COMPARING Iterations:  ****");
      System.out.println("ANY: " + itersAny);
      System.out.println("STA: " + itersStandard);
      System.out.println("****COMPARING COST: ****");
      Scalar anyCost = ((AbstractAnyTrajectoryPlanner) anyTrajectoryPlanner).getGoalCost();
      System.out.println("ANY: " + anyCost);
      Scalar staCost = DoubleScalar.POSITIVE_INFINITY;
      if (standardTrajectoryPlanner != null)
        staCost = standardTrajectoryPlanner.getBest().get().costFromRoot();
      System.out.println("STA: " + staCost);
      System.out.println("*****Finished*****");
      lines = Arrays
          .asList(String.join(",", staTimeDiff.toString(), anyTimeDiff.subtract(staTimeDiff).divide(staTimeDiff).multiply(RealScalar.of(100)).toString(), //
              RealScalar.of(itersAny).subtract(RealScalar.of(itersStandard)).divide(RealScalar.of(itersStandard)).multiply(RealScalar.of(100)).number()
                  .toString(), //
              anyCost.subtract(staCost).divide(staCost).multiply(RealScalar.of(100)).toString()));
      Files.write(path, lines, Charset.forName("UTF-8"), StandardOpenOption.APPEND);
      DebugUtils.heuristicConsistencyCheck((TrajectoryPlanner) anyTrajectoryPlanner);
      if (!owlyFrame.jFrame.isVisible() || itersAny < 1)
        break;
    }
    System.out.println("Finished LOOP");
  }
}
