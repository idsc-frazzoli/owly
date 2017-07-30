// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.rn.RnSimpleCircleGoalManager;
import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class MotionPlanWorker {
  TrajectoryPlannerCallback trajectoryPlannerCallback;
  Tensor goal;
  Thread thread;

  public MotionPlanWorker(TrajectoryPlannerCallback trajectoryPlannerCallback) {
    this.trajectoryPlannerCallback = trajectoryPlannerCallback;
  }

  /** @param head is the old trajectory that is preserved. the planner continues from the last {@link StateTime} in head
   * @param goal
   * @param obstacleQuery */
  public void start(List<TrajectorySample> head, Tensor goal, TrajectoryRegionQuery obstacleQuery) {
    thread = new Thread(new Runnable() {
      @Override
      public void run() {
        System.out.println("start computation");
        Tensor partitionScale = Tensors.vector(6, 6);
        StateIntegrator stateIntegrator = //
            FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 10), 4);
        Collection<Flow> controls = R2Controls.createRadial(23);
        RnSimpleCircleGoalManager rnGoal = //
            new RnSimpleCircleGoalManager(goal, DoubleScalar.of(.2));
        // ---
        TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
            partitionScale, stateIntegrator, controls, obstacleQuery, rnGoal);
        StateTime root = head.get(head.size() - 1).stateTime(); // last statetime in head trajectory
        trajectoryPlanner.insertRoot(root);
        Expand.maxSteps(trajectoryPlanner, 1000); // TODO magic const
        trajectoryPlannerCallback.hasTrajectoryPlanner(head, trajectoryPlanner);
      }
    });
    thread.start();
  }
}
