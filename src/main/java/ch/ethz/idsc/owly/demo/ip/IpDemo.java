// code by jph
package ch.ethz.idsc.owly.demo.ip;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.MidpointIntegrator;
import ch.ethz.idsc.owly.math.region.FreeBoundedIntervalRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;

/** inverted pendulum */
class IpDemo {
  public static void main(String[] args) {
    Tensor eta = Tensors.vector(10, 10, 10, 10);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        MidpointIntegrator.INSTANCE, RationalScalar.of(1, 12), 5);
    StateSpaceModel stateSpaceModel = new IpStateSpaceModel( //
        RealScalar.of(.3), // M
        RealScalar.of(.2), // m
        RealScalar.of(.5), // l
        RealScalar.of(1)); // g;
    Collection<Flow> controls = IpControls.createControls(stateSpaceModel, 2, 10);
    IpGoalManager ipGoalManager = new IpGoalManager( //
        Tensors.vector(2, 0, 0, 0), //
        Tensors.vector(.1, .1, 1, 1));
    TrajectoryRegionQuery obstacleQuery =
        // new EmptyTrajectoryRegionQuery();
        SimpleTrajectoryRegionQuery.timeInvariant( //
            RegionUnion.of( //
                new FreeBoundedIntervalRegion(0, RealScalar.of(-1), RealScalar.of(+3)), // ,
                new FreeBoundedIntervalRegion(2, RealScalar.of(-2), RealScalar.of(+2)) // ,
            ));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, ipGoalManager);
    // ---
    trajectoryPlanner.insertRoot(new StateTime(Array.zeros(4), RealScalar.ZERO));
    // new ExpandGlcFrame(trajectoryPlanner);
    int iters = Expand.maxSteps(trajectoryPlanner, 3000);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyGui.glc(trajectoryPlanner);
  }
}
