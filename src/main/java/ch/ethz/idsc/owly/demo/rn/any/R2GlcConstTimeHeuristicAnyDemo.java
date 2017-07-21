// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnListGoalManager;
import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sin;

enum R2GlcConstTimeHeuristicAnyDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(2.5);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(20, 20);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar runTime = RealScalar.of(0.8);
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    parameters.printResolution();
    System.out.println("DomainSize: 1/Eta: " + parameters.getEta().map(n -> RealScalar.ONE.divide(n)));
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    // Creating Goals
    List<StateTime> goalStateList = new ArrayList<>();
    List<Region> goalRegions = new ArrayList<>();
    Tensor radius = Tensors.vector(0.2, 0.2);
    System.out.println("Goalstates: ");
    for (int i = 0; i < 8; i++) {
      Tensor goal = Tensors.of(RealScalar.of(0.7 * i), Sin.of(RealScalar.of(2 * Math.PI * i / 10)).multiply(RealScalar.of(i * 0.6)));
      System.out.println(goal);
      goalStateList.add(new StateTime(goal, RealScalar.ZERO));
      goalRegions.add(new EllipsoidRegion(goal, radius));
    }
    Tensor heuristicCenter = goalStateList.get(0).x();
    RnListGoalManager rnGoal = new RnListGoalManager(goalRegions, heuristicCenter);
    Region region = new R2NoiseRegion(.1);
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            region));
    AnyPlannerInterface trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    Tensor startState = Tensors.vector(-3, 0);
    trajectoryPlanner.switchRootToState(startState);
    Expand.constTime(trajectoryPlanner, runTime, parameters.getDepthLimit());
    List<StateTime> trajectory = null;
    if (trajectoryPlanner.getBest().isPresent())
      System.out.println("closest Goal found was : " + trajectoryPlanner.getBest().get().state() //
          + " with merit: " + trajectoryPlanner.getBest().get().merit());
    Optional<StateTime> furthestState = trajectoryPlanner.getFurthestGoalState();
    Optional<GlcNode> furthestGoalNode = trajectoryPlanner.getFurthestGoalNode();
    if (furthestState.isPresent()) {
      System.out.println("furthest Goal found was : " + furthestState.get().x() //
          + " with merit: " + furthestGoalNode.get().merit());
      trajectory = GlcNodes.getPathFromRootTo(furthestGoalNode.get());
    }
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
    // -- Anytime loop
    boolean finalGoalFound = false;
    while (!finalGoalFound) {
      Thread.sleep(1);
      long tic = System.nanoTime();
      // Check for final goal
      furthestState = trajectoryPlanner.getFurthestGoalState();
      int deleteIndex = -1;
      if (furthestState.isPresent()) {
        if (goalRegions.get(goalRegions.size() - 1).isMember(furthestState.get().x())) {
          System.out.println("***Last Goal was found***");
          break;
        }
        int index = goalRegions.size();
        while (index > 0) {
          index--;
          if (goalRegions.get(index).isMember(furthestState.get().x())) {
            deleteIndex = index;
            break;
          }
        }
      }
      // TODO JONAS put in class: of goalmanager?
      // -- GOALCHANGE
      final int deleteUntilIndex = deleteIndex; // index of Goal,which was not found yet
      if (deleteIndex < 0)
        System.out.println("No new Goal was found in last run");
      boolean removed = goalRegions.removeIf(gr -> goalRegions.indexOf(gr) < deleteUntilIndex);
      if (removed)
        System.out.println("All Regionparts before " + deleteUntilIndex + " were removed");
      System.out.println("size of goal regions list: " + goalRegions.size());
      rnGoal = new RnListGoalManager(goalRegions, goalStateList.get(7).x());
      // trajectoryPlanner.changeToGoal(rnGoal);
      // -- ROOTCHANGE
      if (trajectory != null) {
        StateTime newRootState = trajectory.get(trajectory.size() > 5 ? 5 : 0);
        int increment = trajectoryPlanner.switchRootToState(newRootState.x());
        parameters.increaseDepthLimit(increment);
      }
      // -- EXPANDING
      int iters2 = Expand.constTime(trajectoryPlanner, runTime, parameters.getDepthLimit());
      if (trajectoryPlanner.getBest().isPresent()) {
        System.out.println("Best Goal found was :" + trajectoryPlanner.getBest().get().state() //
            + " with merit: " + trajectoryPlanner.getBest().get().merit());
      }
      furthestState = trajectoryPlanner.getFurthestGoalState();
      furthestGoalNode = trajectoryPlanner.getFurthestGoalNode();
      if (furthestGoalNode.isPresent()) {
        System.out.println("Furthest Goal found was : " + furthestState.get().x() //
            + " with merit: " + furthestGoalNode.get().merit());
        trajectory = GlcNodes.getPathFromRootTo(furthestGoalNode.get());
        Trajectories.print(trajectory);
      }
      owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
      // --
      long toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After goal switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
    System.out.println("Finished LOOP");
  }
}
