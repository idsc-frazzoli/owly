// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.demo.delta.DeltaControls;
import ch.ethz.idsc.owly.demo.delta.DeltaMinTimeGoalManager;
import ch.ethz.idsc.owly.demo.delta.DeltaStateSpaceModel;
import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.gui.ani.AbstractCircularEntity;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.RungeKutta45Integrator;
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
import ch.ethz.idsc.tensor.red.Norm2Squared;
import ch.ethz.idsc.tensor.sca.Chop;

/** class controls delta using {@link StandardTrajectoryPlanner} */
/* package */ class DeltaEntity extends AbstractCircularEntity {
  public static final Tensor FALLBACK_CONTROL = Tensors.vectorDouble(0, 0).unmodifiable();
  /** preserve 1[s] of the former trajectory */
  private static final Scalar DELAY_HINT = RealScalar.of(2);
  // ---
  /** the constants define the control */
  private static final Scalar U_NORM = RealScalar.of(0.6);
  /** resolution of radial controls */
  private static final int U_SIZE = 15;

  public static StateSpaceModel model(ImageGradient imageGradient) {
    return new DeltaStateSpaceModel(imageGradient, U_NORM);
  }

  /***************************************************/
  private final ImageGradient imageGradient;

  public DeltaEntity(ImageGradient imageGradient, Tensor state) {
    super(new SimpleEpisodeIntegrator(model(imageGradient), EulerIntegrator.INSTANCE, new StateTime(state, RealScalar.ZERO)));
    this.imageGradient = imageGradient;
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return Norm2Squared.between(x, y);
  }

  @Override
  protected final Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  @Override
  public Scalar delayHint() {
    return DELAY_HINT;
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    Tensor eta = eta();
    StateIntegrator stateIntegrator = FixedStateIntegrator.create( //
        RungeKutta45Integrator.INSTANCE, RationalScalar.of(1, 10), 4);
    Collection<Flow> controls = DeltaControls.createControls( //
        new DeltaStateSpaceModel(imageGradient, U_NORM), U_NORM, U_SIZE);
    Scalar u_norm = DeltaControls.maxSpeed(controls);
    GlobalAssert.that(Chop._10.close(u_norm, U_NORM));
    Scalar maxMove = DeltaControls.maxSpeed(controls).add(imageGradient.maxNormGradient());
    GoalInterface goalInterface = DeltaMinTimeGoalManager.create( //
        goal.extract(0, 2), RealScalar.of(.3), maxMove);
    return new StandardTrajectoryPlanner( //
        eta, stateIntegrator, controls, obstacleQuery, goalInterface);
  }

  protected Tensor eta() {
    return Tensors.vector(5, 5).unmodifiable();
  }
}
