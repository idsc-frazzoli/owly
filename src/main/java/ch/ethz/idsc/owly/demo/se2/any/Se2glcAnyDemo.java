// code by jph and jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.demo.se2.Se2DefaultGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2StateSpaceModel;
import ch.ethz.idsc.owly.demo.se2.Se2Utils;
import ch.ethz.idsc.owly.demo.se2.glc.Se2Parameters;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.OptimalAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.gui.Gui;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.HyperplaneRegion;
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

/** (x,y,theta) */
class Se2glcAnyDemo {
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
    StateIntegrator stateIntegrator = FixedStateIntegrator.createDefault(parameters.getdtMax(), //
        parameters.getTrajectorySize());
    // ---
    List<StateTime> trajectory = null;
    System.out.println("1/Domainsize=" + parameters.getEta());
    parameters.printResolution();
    // Se2Controls uses Se2StateSpaceModel
    Collection<Flow> controls = Se2Controls.createControls(Se2Utils.DEGREE(45), parameters.getResolutionInt());
    Se2DefaultGoalManager se2GoalManager = new Se2DefaultGoalManager( //
        Tensors.vector(0, 1), RealScalar.of(Math.PI), //
        DoubleScalar.of(.1), Se2Utils.DEGREE(10));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            RegionUnion.of( //
                new HyperplaneRegion(Tensors.vector(0, -1, 0), RealScalar.of(1.5)), //
                new HyperplaneRegion(Tensors.vector(0, +1, 0), RealScalar.of(4.5)) //
            )));
    // ---
    long tic = System.nanoTime();
    OptimalAnyTrajectoryPlanner trajectoryPlanner = new OptimalAnyTrajectoryPlanner( //
        parameters.getEta(), stateIntegrator, controls, obstacleQuery, se2GoalManager.getGoalInterface());
    // ---
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0, 0));
    int iters = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    System.out.println("After " + iters + " iterations");
    Optional<GlcNode> optional = trajectoryPlanner.getBestOrElsePeek();
    if (optional.isPresent()) {
      trajectory = GlcNodes.getPathFromRootTo(optional.get());
      Trajectories.print(trajectory);
    } else {
      throw new RuntimeException();
    }
    long toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to plan");
    Gui.glc(trajectoryPlanner);
    tic = System.nanoTime();
    // --
    Se2DefaultGoalManager se2GoalManager2 = new Se2DefaultGoalManager( //
        Tensors.vector(-3, 1), RealScalar.of(Math.PI), //
        DoubleScalar.of(0.1), Se2Utils.DEGREE(10));
    StateTime newRootState = null;
    if (trajectory != null)
      newRootState = trajectory.get(trajectory.size() > 3 ? 3 : 0);
    // ---
    trajectoryPlanner.switchRootToState(newRootState.x());
    trajectoryPlanner.changeToGoal(se2GoalManager2.getGoalInterface());
    int iters2 = Expand.maxDepth(trajectoryPlanner, parameters.getDepthLimit());
    // ---
    toc = System.nanoTime();
    System.out.println((toc - tic) * 1e-9 + " Seconds needed to replan");
    System.out.println("After root switch needed " + iters2 + " iterations");
    Gui.glc(trajectoryPlanner);
  }
}
