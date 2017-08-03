// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.Round;

public class MotionPlanWorker {
  private TrajectoryPlannerCallback trajectoryPlannerCallback;
  private Thread thread;
  private volatile boolean isRelevant = true;

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
        long tic = System.nanoTime();
        StateTime root = head.get(head.size() - 1).stateTime(); // last statetime in head trajectory
        trajectoryPlanner.insertRoot(root);
        Expand.maxSteps(trajectoryPlanner, 5000, () -> isRelevant); // magic const
        if (isRelevant) {
          Scalar duration = RealScalar.of((System.nanoTime() - tic) * 1E-9);
          System.out.println("planning: " + duration.map(Round._3) + " [sec]");
          trajectoryPlannerCallback.expandResult(head, trajectoryPlanner);
        }
      }
    });
    thread.start();
  }

  public void flagShutdown() {
    isRelevant = false;
  }
}
