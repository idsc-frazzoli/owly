// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.sca.Ceiling;

/**  */
public class BoundedEpisodeIntegrator extends AbstractEpisodeIntegrator {
  private final Scalar maxStep;

  public BoundedEpisodeIntegrator(StateSpaceModel stateSpaceModel, Integrator integrator, StateTime stateTime, Scalar maxStep) {
    super(stateSpaceModel, integrator, stateTime);
    if (Scalars.lessEquals(maxStep, RealScalar.ZERO))
      throw TensorRuntimeException.of(maxStep);
    this.maxStep = maxStep;
  }

  @Override
  protected List<StateTime> move(Flow flow, Scalar period) {
    Scalar steps = Ceiling.of(period.divide(maxStep));
    return FixedStateIntegrator.create( //
        integrator, period.divide(steps), Scalars.intValueExact(steps)).trajectory(tail(), flow);
  }
}
