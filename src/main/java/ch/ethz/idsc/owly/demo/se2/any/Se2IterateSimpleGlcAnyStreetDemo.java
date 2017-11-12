// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.SimpleAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
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

/** (x,y,theta) */
class Se2IterateSimpleGlcAnyStreetDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(8);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(5, 5, 20 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 6);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), RotationUtils.DEGREE(15));
    // --
    Parameters parameters = new Se2Parameters( //
        resolution, timeScale, depthScale, partitionScale, dtMax, maxIter, stateSpaceModel.getLipschitz());
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        Se2CarIntegrator.INSTANCE, parameters.getdtMax(), parameters.getTrajectorySize());
    // ---
    System.out.println("1/Domainsize=" + parameters.getEta());
    parameters.printResolution();
    // Se2Controls uses Se2StateSpaceModel
    Collection<Flow> controls = Se2Controls.createControls(RotationUtils.DEGREE(45), parameters.getResolutionInt());
    Se2NoHeuristicGoalManager se2GoalManager = new Se2NoHeuristicGoalManager(//
        Tensors.vector(-7, 0, 0), radiusVector);
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.wrap(Arrays.asList( //
            new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(5)), //
            new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(5)) //
        )));
    // ---
    long tic = System.nanoTime();
    AnyPlannerInterface anyPlannerInterface = new SimpleAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    // ---
    anyPlannerInterface.switchRootToState(Tensors.vector(-10, 0, 0));
    int iters = GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    long toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to plan");
    OwlyFrame owlyFrame = OwlyGui.start();
    owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
    // ---
    // --
    for (int iter = 0; iter < 100; iter++) {
      Thread.sleep(500);
      tic = System.nanoTime();
      int index = iter % 4;
      // sawtooth trajectory for goals
      Se2NoHeuristicGoalManager se2GoalManager2 = new Se2NoHeuristicGoalManager(//
          Tensors.vector(-7 + iter, index, 0), radiusVector);
      List<StateTime> trajectory = anyPlannerInterface.trajectoryToBest();
      if (trajectory != null) {
        StateTime newRootState = trajectory.get(trajectory.size() > 2 ? 2 : 0);
        anyPlannerInterface.switchRootToState(newRootState.state());
      } else {
        throw new RuntimeException();
      }
      // --
      anyPlannerInterface.changeToGoal(se2GoalManager2.getGoalInterface());
      int iters2 = GlcExpand.maxDepth(anyPlannerInterface, parameters.getDepthLimit());
      // ---
      toc = System.nanoTime();
      System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
      System.out.println("After root switch needed " + iters2 + " iterations");
      System.out.println("*****Finished*****");
      owlyFrame.setGlc((TrajectoryPlanner) anyPlannerInterface);
      // owlyFrame.configCoordinateOffset(432, 273);
    }
  }
}
