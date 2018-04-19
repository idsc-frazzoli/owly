// code by ynager
package ch.ethz.idsc.owl.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owl.gui.ani.AbstractEntity;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public abstract class WaypointFollowing {
  protected final TrajectoryRegionQuery obstacleQuery;
  protected final AbstractEntity entity;
  private final Tensor waypoints;
  private Scalar distThreshold = DoubleScalar.POSITIVE_INFINITY;
  private boolean isRunning = true;

  public WaypointFollowing(Tensor waypoints, AbstractEntity entity, TrajectoryRegionQuery obstacleQuery) {
    this.waypoints = waypoints;
    this.entity = entity;
    this.obstacleQuery = obstacleQuery;
  }

  /** sets the distance threshold. When the distance from the current state
   * to the current goal is below this threshold, planning to the next goal
   * is initiated
   * 
   * @param distThreshold */
  public final void setDistanceThreshold(Scalar distThreshold) {
    this.distThreshold = distThreshold;
  }

  public final void flagShutdown() {
    isRunning = false;
  }

  /** start planning through waypoints */
  public final void startNonBlocking() {
    Thread thread = new Thread(new Runnable() {
      @Override
      public void run() {
        List<TrajectorySample> head = entity.getFutureTrajectoryUntil(entity.delayHint());
        Tensor goal = waypoints.get(0);
        // start waypoint tracking loop
        int i = 0;
        boolean init = true;
        while (isRunning) {
          Tensor loc = entity.getEstimatedLocationAt(entity.delayHint());
          Scalar dist = entity.distance(loc, goal);
          //
          if (Scalars.lessThan(dist, distThreshold) || init) { // if close enough to current waypoint switch to next
            i = (i + 1) % waypoints.length();
            goal = waypoints.get(i);
            // skip waypoint if covered by obstacle
            if (obstacleQuery.isMember(new StateTime(goal, RealScalar.ZERO))) { // TODO will not work for time dep. obstacles! 
              i = (i + 1) % waypoints.length();
              goal = waypoints.get(i);
            }
            head = entity.getFutureTrajectoryUntil(entity.delayHint());
            planToGoal(head, goal);
            init = false;
          } else {
            try {
              Thread.sleep(50);
            } catch (InterruptedException e) {
              e.printStackTrace();
            }
          }
        }
      }
    });
    thread.start();
  }

  /** starts planning towards a goal
   * 
   * @param TrajectoryPlannerCallback
   * @param head
   * @param goal */
  protected abstract void planToGoal(List<TrajectorySample> head, Tensor goal);
}
