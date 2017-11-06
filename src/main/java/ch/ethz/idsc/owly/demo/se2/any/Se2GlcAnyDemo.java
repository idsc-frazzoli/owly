// code by jph and jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2CarIntegrator;
import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2NoHeuristicGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.AnyPlannerInterface;
import ch.ethz.idsc.owly.glc.core.GlcExpand;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
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
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** (x,y,theta) */
class Se2GlcAnyDemo {
  public static void main(String[] args) throws Exception {
    RationalScalar resolution = (RationalScalar) RealScalar.of(8);
    Scalar timeScale = RealScalar.of(4);
    Scalar depthScale = RealScalar.of(10);
    Tensor partitionScale = Tensors.vector(3, 3, 50 / Math.PI);
    Scalar dtMax = RationalScalar.of(1, 12);
    int maxIter = 2000;
    StateSpaceModel stateSpaceModel = Se2StateSpaceModel.INSTANCE;
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
        Tensors.vector(0, 1, Math.PI), //
        Tensors.vector(0.1, 0.1, 0.1 * Math.PI));
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.of( //
            new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(1.5)), //
            new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4.5)) //
        ));
    // ---
    long tic = System.nanoTime();
    AnyPlannerInterface trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    // ---
    trajectoryPlanner.switchRootToState(Tensors.vector(0, 0, 0));
    int iters = GlcExpand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    List<StateTime> trajectory = trajectoryPlanner.trajectoryToBest();
    long toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to plan");
    OwlyGui.glc((TrajectoryPlanner) trajectoryPlanner);
    tic = System.nanoTime();
    // --
    Se2NoHeuristicGoalManager se2GoalManager2 = new Se2NoHeuristicGoalManager(//
        Tensors.vector(-3, 1, Math.PI), //
        Tensors.vector(0.1, 0.1, 0.1 * Math.PI));
    StateTime newRootState = null;
    if (trajectory != null) {
      newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
      StateTimeTrajectories.print(trajectory);
    } else {
      throw new RuntimeException();
    }
    // ---
    trajectoryPlanner.switchRootToState(newRootState.state());
    trajectoryPlanner.changeToGoal(se2GoalManager2.getGoalInterface());
    int iters2 = GlcExpand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    // ---
    toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
    System.out.println("After root switch needed " + iters2 + " iterations");
    OwlyGui.glc((TrajectoryPlanner) trajectoryPlanner);
  }
}
