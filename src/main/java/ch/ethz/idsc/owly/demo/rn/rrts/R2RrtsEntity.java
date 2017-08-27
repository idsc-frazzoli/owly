// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractRrtsEntity;
import ch.ethz.idsc.owly.gui.ani.PlannerType;
import ch.ethz.idsc.owly.gui.ani.TrajectoryPlannerCallback;
import ch.ethz.idsc.owly.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;

// TODO JAN the redundancy in R2****Entity shows that re-factoring is needed!
public class R2RrtsEntity extends AbstractRrtsEntity {
  private static final Tensor FALLBACK_CONTROL = Tensors.vector(0, 0).unmodifiable();
  /** preserve 1[s] of the former trajectory */
  private static final Scalar DELAY_HINT = RealScalar.ONE;

  // ---
  /** @param state initial position of entity */
  public R2RrtsEntity(Tensor state) {
    super(new SimpleEpisodeIntegrator( //
        SingleIntegratorStateSpaceModel.INSTANCE, //
        EulerIntegrator.INSTANCE, //
        new StateTime(state, RealScalar.ZERO)));
  }

  @Override
  public PlannerType getPlannerType() {
    return PlannerType.RRTS;
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return Norm._2SQUARED.of(x.subtract(y));
  }

  @Override
  protected Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  public Scalar delayHint() {
    return DELAY_HINT;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    throw new RuntimeException(); // TODO API not finalized
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // indicate current position
      Tensor state = getStateTimeNow().state();
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(64, 128, 64, 192));
      graphics.fill(new Ellipse2D.Double(point.getX() - 2, point.getY() - 2, 7, 7));
    }
    { // indicate position 1[s] into the future
      Tensor state = getEstimatedLocationAt(DELAY_HINT);
      Point2D point = owlyLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 128 - 64, 128 + 64));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
  }

  @Override
  public void startPlanner( //
      TrajectoryPlannerCallback trajectoryPlannerCallback, List<TrajectorySample> head, Tensor goal) {
    StateTime tail = head.get(head.size() - 1).stateTime();
    NoiseCircleHelper nch = new NoiseCircleHelper(tail, goal.extract(0, 2));
    nch.plan(300);
    if (nch.trajectory != null) {
      System.out.println("found!");
      trajectoryPlannerCallback.expandResult(head, nch.rrtsPlanner, nch.trajectory);
    }
  }
}
