// code by ynager
package ch.ethz.idsc.owl.glc.adapter;

import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.AbstractEntity;
import ch.ethz.idsc.owl.gui.ani.MotionPlanWorker;
import ch.ethz.idsc.owl.gui.ani.TrajectoryPlannerCallback;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.Tensor;

public class GlcWaypointFollowing extends WaypointFollowing {
  protected final TrajectoryPlannerCallback trajectoryPlannerCallback;
  private MotionPlanWorker mpw = null;

  public GlcWaypointFollowing( //
      Tensor waypoints, AbstractEntity entity, TrajectoryPlannerCallback trajectoryPlannerCallback, //
      TrajectoryRegionQuery obstacleQuery) {
    super(waypoints, entity, obstacleQuery);
    this.trajectoryPlannerCallback = trajectoryPlannerCallback;
  }

  @Override
  protected void planToGoal(List<TrajectorySample> head, Tensor goal) {
    if (Objects.nonNull(mpw)) {
      mpw.flagShutdown();
      mpw = null;
    }
    TrajectoryPlanner trajectoryPlanner = entity.createTrajectoryPlanner( //
        Objects.requireNonNull(super.obstacleQuery), goal);
    mpw = new MotionPlanWorker(trajectoryPlannerCallback);
    mpw.start(head, trajectoryPlanner);
  }
}
