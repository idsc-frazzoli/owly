// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.Scalar;

/** {@link SimpleEpisodeIntegrator} takes the largest possible time step for integration.
 * 
 * implementation is fast and should only be applied for simple {@link StateSpaceModel}s */
public class SimpleEpisodeIntegrator extends AbstractEpisodeIntegrator {
  public SimpleEpisodeIntegrator(StateSpaceModel stateSpaceModel, Integrator integrator, StateTime stateTime) {
    super(stateSpaceModel, integrator, stateTime);
  }

  @Override
  protected List<StateTime> move(Flow flow, Scalar period) {
    return FixedStateIntegrator.create(integrator, period, 1).trajectory(stateTime, flow);
  }
}
