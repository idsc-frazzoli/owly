// code by ynager

package ch.ethz.idsc.owl.glc.adapter;

import java.awt.Color;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owl.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.ani.AbstractAnyEntity;
import ch.ethz.idsc.owl.gui.ani.AbstractEntity;
import ch.ethz.idsc.owl.gui.ani.AbstractRrtsEntity;
import ch.ethz.idsc.owl.gui.ani.MotionPlanWorker;
import ch.ethz.idsc.owl.gui.ani.OwlyAnimationFrame;
import ch.ethz.idsc.owl.gui.ren.PointRender;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class WaypointFollowing {
  private Tensor waypoints;
  private AbstractEntity entity;
  private OwlyAnimationFrame owlyAnimationFrame;
  private TrajectoryRegionQuery obstacleQuery = null;
  private Scalar distThreshold = RealScalar.of(1);

  public WaypointFollowing(Tensor waypoints, AbstractEntity entity, OwlyAnimationFrame owlyAnimationFrame) {
    this.waypoints = waypoints;
    this.entity = entity;
    this.owlyAnimationFrame = owlyAnimationFrame;
    {
      RenderInterface pointRenderInterface = new PointRender(waypoints, 5, Color.black);
      owlyAnimationFrame.addBackground(pointRenderInterface);
    }
  }

  /** modifies the obstacle region
   * (so far only relevant for the standard planner)
   * 
   * @param obstacleQuery */
  public void setObstacleQuery(TrajectoryRegionQuery obstacleQuery) { // only used for glc
    this.obstacleQuery = obstacleQuery;
  }

  /** sets the distance threshold. When the distance from the current state
   * to the current goal is below this threshold, planning to the next goal 
   * is initiated
   * 
   * @param distThreshold */
  public void setDistanceThreshold(Scalar distThreshold) {
    this.distThreshold = distThreshold;
  }

  /** Start looping through waypoints */
  public void start() {
    MotionPlanWorker mpw = null;
    List<TrajectorySample> head = entity.getFutureTrajectoryUntil(entity.delayHint());
    Tensor goal = waypoints.get(0);
    //
    // start waypoint tracking loop
    int i = 0;
    boolean init = true;
    while (true) {
      Tensor loc = entity.getEstimatedLocationAt(entity.delayHint());
      Scalar dist = entity.distance(loc, goal).abs();
      //
      if (Scalars.lessThan(dist, distThreshold) || init) { // if close enough to current waypoint switch to next
        // shut down mpw
        if (Objects.nonNull(mpw)) {
          mpw.flagShutdown();
          mpw = null;
        }
        i = (i + 1) % waypoints.length();
        goal = waypoints.get(i);
        head = entity.getFutureTrajectoryUntil(entity.delayHint());
        //
        switch (entity.getPlannerType()) {
        case STANDARD: {
          TrajectoryPlanner trajectoryPlanner = //
              entity.createTrajectoryPlanner(obstacleQuery, goal);
          mpw = new MotionPlanWorker(owlyAnimationFrame.trajectoryPlannerCallback);
          mpw.start(head, trajectoryPlanner);
          break;
        }
        case ANY: {
          AbstractAnyEntity abstractAnyEntity = (AbstractAnyEntity) entity;
          abstractAnyEntity.switchToGoal(owlyAnimationFrame.trajectoryPlannerCallback, head, goal);
          break;
        }
        case RRTS: {
          AbstractRrtsEntity abstractRrtsEntity = (AbstractRrtsEntity) entity;
          abstractRrtsEntity.startPlanner(owlyAnimationFrame.trajectoryPlannerCallback, head, goal);
          break;
        }
        default:
          throw new RuntimeException();
        }
        init = false;
      } else {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }
}
