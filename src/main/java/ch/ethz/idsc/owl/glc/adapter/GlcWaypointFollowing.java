package ch.ethz.idsc.owl.glc.adapter;

import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.gui.ani.AbstractEntity;
import ch.ethz.idsc.owl.gui.ani.MotionPlanWorker;
import ch.ethz.idsc.owl.gui.ani.TrajectoryPlannerCallback;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.Tensor;

public class GlcWaypointFollowing extends WaypointFollowing {
  private TrajectoryRegionQuery obstacleQuery;

  public GlcWaypointFollowing(Tensor waypoints, AbstractEntity entity, TrajectoryPlannerCallback trajectoryPlannerCallback,
      TrajectoryRegionQuery obstacleQuery) {
    super(waypoints, entity, trajectoryPlannerCallback);
    this.obstacleQuery = obstacleQuery;
  }

  @Override
  protected void planToGoal(List<TrajectorySample> head, Tensor goal) {
    MotionPlanWorker mpw = null;
    GlobalAssert.that(Objects.nonNull(obstacleQuery));
    TrajectoryPlanner trajectoryPlanner = //
        entity.createTrajectoryPlanner(obstacleQuery, goal);
    mpw = new MotionPlanWorker(super.trajectoryPlannerCallback);
    mpw.start(head, trajectoryPlanner);
  }
}
