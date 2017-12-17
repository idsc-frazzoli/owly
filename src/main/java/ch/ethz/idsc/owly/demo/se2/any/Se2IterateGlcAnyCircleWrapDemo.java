// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owl.glc.adapter.GlcExpand;
import ch.ethz.idsc.owl.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.any.AbstractAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.any.AnyPlannerInterface;
import ch.ethz.idsc.owl.glc.any.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.gui.ani.OwlyFrame;
import ch.ethz.idsc.owl.gui.ani.OwlyGui;
import ch.ethz.idsc.owl.math.CoordinateWrap;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.region.HyperplaneRegion;
import ch.ethz.idsc.owl.math.region.InvertedRegion;
import ch.ethz.idsc.owl.math.region.RegionUnion;
import ch.ethz.idsc.owl.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owl.math.state.StateIntegrator;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.demo.se2.CarStandardFlows;
import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2LateralAcceleration;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Wrap;
import ch.ethz.idsc.owly.demo.se2.Se2WrapGoalManagerExt;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** (x,y,theta) */
class Se2IterateGlcAnyCircleWrapDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(6);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(5, 5, 20 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 12);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    Scalar circleRadius = RealScalar.of(3);
    Scalar goalAngle = RealScalar.of(0);
    Tensor goal = Tensors.of(Cos.of(goalAngle).multiply(circleRadius), //
        Sin.of(goalAngle).multiply(circleRadius), goalAngle.subtract(RealScalar.of(Math.PI * 0.5)));
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), Degree.of(15));
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        Se2CarIntegrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    // ---
    parameters.printResolution();
    CarStandardFlows carConfig = new CarStandardFlows(RealScalar.ONE, Degree.of(45));
    Collection<Flow> controls = carConfig.getFlows(parameters.getResolutionInt());
    CoordinateWrap coordinateWrap = new Se2Wrap(Tensors.vector(1, 1, 1));
    GoalInterface se2GoalManager = MultiCostGoalAdapter.of( //
        new Se2WrapGoalManagerExt(coordinateWrap, new Se2MinTimeGoalManager(goal, radiusVector, controls)).getGoalInterface(), //
        Arrays.asList(Se2LateralAcceleration.COSTFUNCTION));
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.wrap(Arrays.asList( //
            new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(1, 1, Double.POSITIVE_INFINITY)), //
            new InvertedRegion(new EllipsoidRegion(Tensors.vector(0, 0, 0), Tensors.vector(5, 5, Double.POSITIVE_INFINITY))), //
            new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(4)), //
            new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4)) //
        )));
    // ---
    Scalar tic = RealScalar.of(System.nanoTime());
    AnyPlannerInterface anyPlannerInterface = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager);
    // ---
    anyPlannerInterface.switchRootToState(new StateTime(Tensors.vector(0, 3, 0), RealScalar.ZERO));
    OwlyFrame owlyFrame = OwlyGui.start();
    Scalar toc = RealScalar.of(System.nanoTime());
    int iters = GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    System.out.println(toc.subtract(tic).multiply(RealScalar.of(1e-9)) + " Seconds needed to plan");
    owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
    // ---
    // --
    int iter = 0;
    Scalar timeSum = RealScalar.of(0);
    boolean goalFound = false;
    int expandIter = 0;
    while (iter < 20) {
      // while (owlyFrame.jFrame.isVisible()) {
      Scalar delay = RealScalar.of(0);
      Thread.sleep(1000);
      tic = RealScalar.of(System.nanoTime());
      iter++;
      // --
      List<StateTime> trajectory = anyPlannerInterface.trajectoryToBest();
      if (trajectory != null) {
        StateTime newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
        int increment = anyPlannerInterface.switchRootToState(newRootState);
        parameters.increaseDepthLimit(increment);
      } else {
        throw new RuntimeException();
      }
      owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
      Thread.sleep(delay.number().intValue() / 2);
      // --
      goalFound = Se2CircleAnyGoalSwitch.switchToNextCircularGoal(//
          (AbstractAnyTrajectoryPlanner) anyPlannerInterface, //
          iter, //
          controls, //
          parameters);
      Thread.sleep(delay.number().intValue() / 2);
      // --
      if (!goalFound)
        expandIter = GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
      // ---
      toc = RealScalar.of(System.nanoTime());
      StateTimeTrajectories.print(trajectory);
      timeSum = toc.subtract(tic).multiply(RealScalar.of(1e-9)).add(timeSum);
      System.out.println((iter + 1) + ". iteration took: " + toc.subtract(tic).multiply(RealScalar.of(1e-9)) + "s");
      System.out.println("Average: " + timeSum.divide(RealScalar.of(iter + 1)));
      System.out.println("After root switch needed " + expandIter + " iterations");
      System.out.println("*****Finished*****");
      System.out.println("");
      owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
      // owlyFrame.configCoordinateOffset(432, 273);
      iter++;
    }
  }
}
