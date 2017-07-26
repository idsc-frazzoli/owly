// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.rn.R2NoiseRegion;
import ch.ethz.idsc.owly.demo.rn.RnSimpleEllipsoidGoalManager;
import ch.ethz.idsc.owly.demo.util.R2Controls;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.Expand;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class MotionPlanWorker implements Runnable {
  TrajectoryPlannerCallback trajectoryPlannerCallback;
  Tensor goal;
  Thread thread;

  public MotionPlanWorker(TrajectoryPlannerCallback trajectoryPlannerCallback) {
    this.trajectoryPlannerCallback = trajectoryPlannerCallback;
  }

  public void start(Tensor goal) {
    this.goal = goal;
    thread = new Thread(this);
    thread.start();
  }

  @Override
  public void run() {
    System.out.println("start computation");
    Tensor partitionScale = Tensors.vector(6, 6);
    Region region = new R2NoiseRegion(.3);
    StateIntegrator stateIntegrator = //
        FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RationalScalar.of(1, 10), 4);
    Collection<Flow> controls = R2Controls.createRadial(23);
    RnSimpleEllipsoidGoalManager rnGoal = //
        new RnSimpleEllipsoidGoalManager(goal, DoubleScalar.of(.2));
    TrajectoryRegionQuery obstacleQuery = //
        new SimpleTrajectoryRegionQuery(new TimeInvariantRegion( //
            region));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        partitionScale, stateIntegrator, controls, obstacleQuery, rnGoal);
    trajectoryPlanner.insertRoot(Tensors.vector(0, 0));
    Expand.maxSteps(trajectoryPlanner, 1000);
    trajectoryPlannerCallback.hasTrajectoryPlanner(trajectoryPlanner);
  }
}
