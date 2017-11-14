// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.data.Stopwatch;
import ch.ethz.idsc.owly.demo.rn.R2Config;
import ch.ethz.idsc.owly.demo.rn.R2NoiseCostFunction;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.RnMinDistSphericalGoalManager;
import ch.ethz.idsc.owly.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.OwlyFrame;
import ch.ethz.idsc.owly.gui.ani.OwlyGui;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
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

/** expands: 1491
 * computation time: 0.876993604 */
enum R2NoiseDemo {
  ;
  public static void main(String[] args) {
    Tensor partitionScale = Tensors.vector(8, 8);
    final Scalar threshold = RealScalar.of(0.1);
    Region<Tensor> region = new R2NoiseRegion(threshold);
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 12), 4);
    R2Config r2Config = new R2Config(RealScalar.ONE);
    Collection<Flow> controls = r2Config.createRadial(23);
    final Tensor center = Tensors.vector(10, 0);
    final Scalar radius = DoubleScalar.of(0.2);
    SphericalRegion sphericalRegion = new SphericalRegion(center, radius);
    GoalInterface goalInterface = MultiCostGoalAdapter.of( //
        new RnMinDistSphericalGoalManager(sphericalRegion), //
        Arrays.asList(new R2NoiseCostFunction(threshold.subtract(RealScalar.of(0.3)))));
    TrajectoryRegionQuery obstacleQuery = SimpleTrajectoryRegionQuery.timeInvariant(region);
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, goalInterface);
    trajectoryPlanner.insertRoot(new StateTime(Array.zeros(2), RealScalar.ZERO));
    Stopwatch stopwatch = Stopwatch.started();
    int iters = Expand.maxSteps(trajectoryPlanner, 10000);
    System.out.println(iters + " " + stopwatch.display_seconds());
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyFrame owlyFrame = OwlyGui.glc(trajectoryPlanner);
    owlyFrame.addBackground(RegionRenders.create(sphericalRegion));
    owlyFrame.configCoordinateOffset(100, 300);
  }
}
