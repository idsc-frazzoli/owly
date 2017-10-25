// code by jph
package ch.ethz.idsc.owly.demo.rnd.glc;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.data.Stopwatch;
import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rnd.R2dControls;
import ch.ethz.idsc.owly.demo.rnd.RndOrRegion;
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
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

enum R2dNoiseDemo {
  ;
  public static void main(String[] args) {
    Tensor partitionScale = Tensors.vector(6, 6, 6, 6);
    final Scalar threshold = RealScalar.of(.2);
    Region region = RndOrRegion.common(new R2NoiseRegion(threshold));
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 8), 3);
    Collection<Flow> controls = R2dControls.createRadial(5);
    final Tensor center = Tensors.vector(3, 0);
    final Scalar radius = DoubleScalar.of(.2);
    GoalInterface goalInterface = //
        new RndMinDistSphericalGoalManager(center, radius);
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, goalInterface);
    // Trajectories.s
    Tensor root_x = Tensors.vector(0, 0, 0, 0);
    boolean root_free = obstacleQuery.isDisjoint(Collections.singletonList(new StateTime(root_x, RealScalar.ONE)));
    GlobalAssert.that(root_free);
    trajectoryPlanner.insertRoot(root_x);
    Stopwatch stopwatch = Stopwatch.started();
    int iters = Expand.maxSteps(trajectoryPlanner, 1000);
    System.out.println(iters + " " + stopwatch.display_seconds());
    Optional<GlcNode> optional = trajectoryPlanner.getBest();
    if (optional.isPresent()) {
      List<StateTime> trajectory = GlcNodes.getPathFromRootTo(optional.get());
      StateTimeTrajectories.print(trajectory);
    }
    OwlyFrame owlyFrame = OwlyGui.glc(trajectoryPlanner);
    owlyFrame.configCoordinateOffset(100, 300);
  }
}
