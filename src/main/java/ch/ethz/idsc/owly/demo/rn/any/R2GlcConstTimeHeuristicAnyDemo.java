// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnListGoalManager;
import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
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
    Scalar timeScale = RealScalar.of(2);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(30, 30);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar runTime = RealScalar.of(1);
    Scalar lipschitz = RealScalar.ONE;
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    List<StateTime> goalStateList = new ArrayList<>();
    List<Region> goalRegions = new ArrayList<>();
    Tensor radius = Tensors.vector(0.3, 0.3);
    for (int i = 0; i < 8; i++) {
      Scalar angle = RealScalar.of(i).multiply(RealScalar.of(Math.PI / 4));
      Tensor goal = Tensors.of(RealScalar.of(i), Sin.of(angle).multiply(RealScalar.of(0.75 * i)));
      goalStateList.add(new StateTime(goal, RealScalar.ZERO));
      goalRegions.add(new EllipsoidRegion(goal, radius));
    }
    TimeInvariantRegion goalRegion = new TimeInvariantRegion(RegionUnion.of(goalRegions));
    Tensor heuristicCenter = goalStateList.get(0).x();
    RnListGoalManager rnGoal = new RnListGoalManager(goalRegion, heuristicCenter);
    // RnGoalManager rnGoal = new RnGoalManager(goal, DoubleScalar.of(.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    // TrajectoryRegionQuery obstacleQuery = //
    // new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
    // RegionUnion.of( //
    // new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(RealScalar.of(2))) //
    // , new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(RealScalar.of(6))))
    // , RnPointclouds.createRandomRegion(30, Tensors.vector(12, 12), Tensors.vector(0, 0), RealScalar.of(0.6))//
    // )));
    // --
    TrajectoryRegionQuery obstacleQuery = new SimpleTrajectoryRegionQuery(new EmptyRegion());
    TrajectoryRegionQuery obstacleQuery = new EmptyTrajectoryRegionQuery();
    AnyPlannerInterface trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    Tensor startNode = Tensors.vector(-3, 0);
    trajectoryPlanner.switchRootToState(startNode);
    Expand.constTime(trajectoryPlanner, runTime, parameters.getDepthLimit());
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    boolean finalGoalFound = false;
    while (!finalGoalFound) {
      Thread.sleep(1);
      long tic = System.nanoTime();
      // Check for final goal
      Optional<GlcNode> best = trajectoryPlanner.getBest();
      if (trajectoryPlanner.getBest().isPresent()) {
        Iterator<Region> regionIterator = goalRegions.iterator();
        Region temp = new EmptyRegion();
        while (regionIterator.hasNext())
          temp = regionIterator.next();
        if (temp.isMember(best.get().state())) {
          System.out.println("***Last Goal was found***");
          break;
        }
      }
      // TODO JONAS put in class:
      // -- GOALCHANGE
      int index = 0;
      for (index = goalRegions.size(); index < 0; index--) { // going backwards through Goaltrajectory until found furthest (first) foundGoal
        if (goalRegions.get(index).isMember(best.get().state()))
          break;
      }
      if (index == 0)
        System.out.println("No new Goal was found in last run");
      final int deleteUntilIndex = index + 1; // index of Goal,which was not found yet
      goalRegions.removeIf(gr -> goalRegions.indexOf(gr) < deleteUntilIndex); // Deleting all goals before the first not found
      TimeInvariantRegion goalRegion2 = new TimeInvariantRegion(RegionUnion.of(goalRegions)); // modified (smaller) GoalRegion
      RnListGoalManager rnGoal2 = new RnListGoalManager(goalRegion2, heuristicCenter);
      trajectoryPlanner.changeToGoal(rnGoal2);
      // -- ROOTCHANGE
      List<StateTime> trajectory = trajectoryPlanner.trajectoryToBest();
      // StateTime newRootState = trajectory.get(trajectory.size() > 5 ? 5 : 0);
      // int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      // parameters.increaseDepthLimit(increment);
      // -- EXPANDING
      int iters2 = Expand.constTime(trajectoryPlanner, runTime, parameters.getDepthLimit());
      owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
      Trajectories.print(trajectory);
      // --
      long toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
    System.out.println("Finished LOOP");
  }
}
