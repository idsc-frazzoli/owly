// code by jph
package ch.ethz.idsc.owly.demo.psu.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.demo.psu.PsuControls;
import ch.ethz.idsc.owly.demo.psu.PsuGoalManager;
import ch.ethz.idsc.owly.demo.psu.PsuStateSpaceModel;
import ch.ethz.idsc.owly.demo.psu.PsuWrap;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.AbstractCircularEntity;
import ch.ethz.idsc.owly.math.StateTimeTensorFunction;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.owly.math.flow.RungeKutta4Integrator;
import ch.ethz.idsc.owly.math.state.EmptyTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.FixedStateIntegrator;
import ch.ethz.idsc.owly.math.state.SimpleEpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/* package */ class PsuEntity extends AbstractCircularEntity {
  private static final Integrator INTEGRATOR = RungeKutta4Integrator.INSTANCE;
  private static final Tensor FALLBACK_CONTROL = Tensors.vector(0).unmodifiable();
  /** preserve 1[s] of the former trajectory */
  private static final Scalar DELAY_HINT = RealScalar.ONE;

  public PsuEntity() {
    super(new SimpleEpisodeIntegrator( //
        PsuStateSpaceModel.INSTANCE, //
        INTEGRATOR, //
        new StateTime(Tensors.vector(0, 0), RealScalar.ZERO)));
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return PsuWrap.INSTANCE.distance(x, y);
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
    Tensor eta = Tensors.vector(6, 8);
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        INTEGRATOR, RationalScalar.of(1, 4), 5);
    Collection<Flow> controls = PsuControls.createControls(0.2, 6);
    PsuWrap psuWrap = PsuWrap.INSTANCE;
    GoalInterface goalInterface = PsuGoalManager.of( //
        psuWrap, psuWrap.represent(goal.extract(0, 2)), RealScalar.of(0.2));
    // ---
    TrajectoryPlanner trajectoryPlanner = new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, EmptyTrajectoryRegionQuery.INSTANCE, goalInterface);
    trajectoryPlanner.represent = StateTimeTensorFunction.state(psuWrap::represent);
    return trajectoryPlanner;
  }
}
