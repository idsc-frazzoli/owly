// code by jph
package ch.ethz.idsc.owl.math.state;

import java.util.List;

import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.flow.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.sca.Ceiling;
import ch.ethz.idsc.tensor.sca.Sign;

/** integrates along given flow with time steps that do not exceed a predefined threshold */
public class BoundedEpisodeIntegrator extends AbstractEpisodeIntegrator {
  private final Scalar maxStep;

  /** @param stateSpaceModel
   * @param integrator
   * @param stateTime initial state
   * @param maxStep in time that given integrator applies */
  public BoundedEpisodeIntegrator(StateSpaceModel stateSpaceModel, Integrator integrator, StateTime stateTime, Scalar maxStep) {
    super(stateSpaceModel, integrator, stateTime);
    if (Sign.isNegativeOrZero(maxStep))
      throw TensorRuntimeException.of(maxStep);
    this.maxStep = maxStep;
  }

  @Override // from AbstractEpisodeIntegrator
  protected List<StateTime> move(Flow flow, Scalar period) {
    Scalar steps = Ceiling.of(period.divide(maxStep));
    return FixedStateIntegrator.create( //
        integrator, period.divide(steps), Scalars.intValueExact(steps)).trajectory(tail(), flow);
  }
}
