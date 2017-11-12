// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnNoHeuristicCircleGoalManager;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
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
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.lie.AngleVector;

enum R2GlcAnyCircleDemo {
  ;
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(15);
    Scalar timeScale = RealScalar.of(2);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(30, 30);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Scalar circleRadius = RealScalar.of(6);
    Scalar goalAngle = RealScalar.ZERO;
    Tensor goal = AngleVector.of(goalAngle).multiply(circleRadius);
    Scalar goalRadius = DoubleScalar.of(.25);
    System.out.println("Goal is: " + goal);
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, parameters.getdtMax(), //
        parameters.getTrajectorySize());
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt()); // max (grad(F)) ==1
    RnNoHeuristicCircleGoalManager rnGoal = new RnNoHeuristicCircleGoalManager(goal, goalRadius);
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.wrap(Arrays.asList( //
            new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(circleRadius).multiply(RealScalar.of(0.5))) //
            , new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(circleRadius).multiply(RealScalar.of(2)))) //
            // , RnPointclouds.createRandomRegion(30, Tensors.vector(12, 12), Tensors.vector(0, 0), RealScalar.of(0.6)) //
            , new R2NoiseRegion(RealScalar.of(.2))//
        )));
    // ---
    AnyPlannerInterface anyPlannerInterface = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    anyPlannerInterface.switchRootToState(Tensors.vector(0, 1).multiply(circleRadius));
    GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
    List<StateTime> trajectory = anyPlannerInterface.trajectoryToBest();
    // StateTimeTrajectories.print(trajectory);
    // AnimationWriter gsw = AnimationWriter.of(UserHome.Pictures("R2_Circle_Gif.gif"), 250);
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
    for (int iter = 1; iter < 100; iter++) {
      Thread.sleep(100);
      // -- ROOTCHANGE
      long tic = System.nanoTime();
      if (trajectory.size() > 0) {
        // --
        StateTime newRootState = trajectory.get(trajectory.size() > 5 ? 5 : 0);
        int increment = anyPlannerInterface.switchRootToState(newRootState.state());
        parameters.increaseDepthLimit(increment);
      }
      // -- GOALCHANGE
      List<StateTime> goalStateList = new ArrayList<>();
      do {
        goalStateList.clear();
        goalAngle = goalAngle.subtract(RealScalar.of(0.1 * Math.PI));
        goal = AngleVector.of(goalAngle).multiply(circleRadius);
        StateTime goalState = new StateTime(goal, RealScalar.ZERO);
        goalStateList.add(goalState);
      } while (!obstacleQuery.isDisjoint(goalStateList));
      GoalInterface rnGoal2 = new RnNoHeuristicCircleGoalManager(goal, goalRadius);
      System.out.println("Switching to Goal:" + goal);
      Scalar goalSearchHelperRadius = goalRadius.add(RealScalar.ONE).multiply(RealScalar.of(2));
      Region<Tensor> goalSearchHelper = new EllipsoidRegion(goal, Array.of(l -> goalSearchHelperRadius, goal.length()));
      anyPlannerInterface.changeToGoal(rnGoal2, goalSearchHelper);
      // -- EXPANDING
      int iters2 = GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
      trajectory = anyPlannerInterface.trajectoryToBest();
      owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
      // StateTimeTrajectories.print(trajectory);
      // gsw.append(owlyFrame.offscreen());
      // --
      long toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
    // int repeatLast = 6;
    // while (0 < repeatLast--)
    // gsw.append(owlyFrame.offscreen());
    // gsw.close();
    // System.out.println("created gif");
    System.out.println("Finished LOOP");
  }
}
