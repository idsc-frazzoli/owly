// code by jl
package ch.ethz.idsc.owly.demo.rn.any;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.rn.R2Parameters;
import ch.ethz.idsc.owly.demo.rn.RnGoalManager;
import ch.ethz.idsc.owly.demo.rn.RnPointclouds;
import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.demo.util.UserHome;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.gui.OwlyFrame;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.GifSequenceWriter;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class R2GlcAnyCircleDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(10);
    Scalar timeScale = RealScalar.of(2);
    Scalar depthScale = RealScalar.of(100);
    Tensor partitionScale = Tensors.vector(30, 30);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    Scalar lipschitz = RealScalar.ONE;
    Scalar circleRadius = RealScalar.of(6);
    Scalar goalAngle = RealScalar.ZERO;
    Tensor goal = Tensors.of(Cos.of(goalAngle), Sin.of(goalAngle)).multiply(circleRadius);
    System.out.println("Goal is: " + goal);
    Parameters parameters = new R2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, lipschitz);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(new EulerIntegrator(), parameters.getdtMax(), //
        parameters.getTrajectorySize());
    Collection<Flow> controls = R2Controls.createRadial(parameters.getResolutionInt());
    RnGoalManager rnGoal = new RnGoalManager(goal, DoubleScalar.of(.25));
    // performance depends on heuristic: zeroHeuristic vs rnGoal
    // Heuristic heuristic = new ZeroHeuristic(); // rnGoal
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(circleRadius).multiply(RealScalar.of(0.5))) //
                , new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0), Tensors.vector(1, 1).multiply(circleRadius).multiply(RealScalar.of(2)))),
                RnPointclouds.createRandomRegion(30, Tensors.vector(12, 12), Tensors.vector(0, 0), RealScalar.of(0.6))//
            )));
    // --
    AnyPlannerInterface trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.switchRootToState(Tensors.vector(0, 1).multiply(circleRadius));
    GifSequenceWriter gsw = GifSequenceWriter.of(UserHome.Pictures("R2_Circle_Gif.gif"), 250);
    OwlyFrame owlyFrame = Gui.start();
    owlyFrame.configCoordinateOffset(400, 400);
    owlyFrame.jFrame.setBounds(0, 0, 800, 800);
    for (int iter = 1; iter < 31; iter++) {
      Thread.sleep(1);
      long tic = System.nanoTime();
      List<StateTime> trajectory = trajectoryPlanner.TrajectoryToGoal();
      // -- GOAL change
      List<StateTime> goalStateList = new ArrayList<>();
      do {
        goalStateList.clear();
        goalAngle = goalAngle.subtract(RealScalar.of(0.1 * Math.PI));
        goal = Tensors.of(Cos.of(goalAngle), Sin.of(goalAngle)).multiply(circleRadius);
        StateTime goalState = new StateTime(goal, RealScalar.ZERO);
        goalStateList.add(goalState);
      } while (!obstacleQuery.isDisjoint(goalStateList));
      RnGoalManager rnGoal2 = new RnGoalManager(goal, DoubleScalar.of(.25));
      System.out.println("Switching to Goal:" + goal);
      trajectoryPlanner.changeToGoal(rnGoal2);
      // -- ROOTCHANGE
      StateTime newRootState = trajectory.get(trajectory.size() > 5 ? 5 : 0);
      int increment = trajectoryPlanner.switchRootToState(newRootState.x());
      parameters.increaseDepthLimit(increment);
      // -- EXPANDING
      int iters2 = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
      owlyFrame.setGlc((TrajectoryPlanner) trajectoryPlanner);
      Trajectories.print(trajectory);
      gsw.append(owlyFrame.offscreen());
      // --
      long toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      if (!owlyFrame.jFrame.isVisible())
        break;
    }
    int repeatLast = 6;
    while (0 < repeatLast--)
      gsw.append(owlyFrame.offscreen());
    gsw.close();
    System.out.println("created gif");
    System.out.println("Finished LOOP");
  }
}
