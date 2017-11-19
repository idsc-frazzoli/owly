// code by jph
package ch.ethz.idsc.owly.gui.ani;

import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.subare.core.DiscreteModel;
import ch.ethz.idsc.subare.core.Policy;
import ch.ethz.idsc.subare.core.util.PolicyWrap;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public abstract class PolicyEntity implements AnimationInterface, DiscreteModel {
  private final EpisodeIntegrator episodeIntegrator;
  Policy policy;

  public PolicyEntity(EpisodeIntegrator episodeIntegrator) {
    this.episodeIntegrator = episodeIntegrator;
  }

  public abstract Tensor represent(StateTime stateTime);

  @Override // from AnimationInterface
  public final void integrate(Scalar now) {
    // implementation does not require that current position is perfectly located on trajectory
    // Tensor u = fallbackControl(); // default control
    StateTime stateTime = getStateTimeNow();
    Tensor state = represent(stateTime); // may be augmented state time, and/or observation etc.
    PolicyWrap policyWrap = new PolicyWrap(policy);
    Tensor actions = null; // FIXME
    Tensor u = policyWrap.next(state, actions);
    episodeIntegrator.move(u, now);
  }

  /** @return control vector to feed the episodeIntegrator in case no planned trajectory is available */
  protected abstract Tensor fallbackControl();

  public final StateTime getStateTimeNow() {
    return episodeIntegrator.tail();
  }
}
