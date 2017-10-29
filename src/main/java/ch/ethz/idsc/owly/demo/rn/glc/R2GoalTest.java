// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.R2Controls;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2GoalTest {
  ;
  public static void main(String[] args) {
    Tensor partitionScale = Tensors.vector(3.5, 4);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 8), 5);
    Collection<Flow> controls = R2Controls.createRadial(20);
    GoalInterface goalInterface = RnMinDistSphericalGoalManager.create(Tensors.vector(5, 0), DoubleScalar.of(0.5));
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant( //
        RegionUnion.of( //
            new EllipsoidRegion(Tensors.vector(3, 3), Tensors.vector(2, 2)), //
            new EllipsoidRegion(Tensors.vector(2.5, 0), Tensors.vector(2, 1.5)) //
        ));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters);
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyGui.glc(trajectoryPlanner);
  }
}
