// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;

public class MotionPlanWorker {
  TrajectoryPlannerCallback trajectoryPlannerCallback;
  Tensor goal;
  Thread thread;

  public MotionPlanWorker(TrajectoryPlannerCallback trajectoryPlannerCallback) {
    this.trajectoryPlannerCallback = trajectoryPlannerCallback;
  }

  /** the planner motion plans from the last {@link StateTime} in head
   * 
   * @param head non-empty trajectory
   * @param trajectoryPlanner */
  public void start(List<TrajectorySample> head, TrajectoryPlanner trajectoryPlanner) {
    thread = new Thread(new Runnable() {
      @Override
      public void run() {
        System.out.println("start computation");
        StateTime root = head.get(head.size() - 1).stateTime(); // last statetime in head trajectory
        trajectoryPlanner.insertRoot(root);
        // TODO define API do allow entity to abort/continue expand; right now: magic const
        Expand.maxSteps(trajectoryPlanner, 4000);
        trajectoryPlannerCallback.expandResult(head, trajectoryPlanner);
      }
    });
    thread.start();
  }
}
