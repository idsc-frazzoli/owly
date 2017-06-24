// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

abstract class AbstractEpisodeIntegrator implements EpisodeIntegrator {
  final StateSpaceModel stateSpaceModel;
  final Integrator integrator;
  StateTime stateTime;

  AbstractEpisodeIntegrator(StateSpaceModel stateSpaceModel, Integrator integrator, StateTime stateTime) {
    this.stateSpaceModel = stateSpaceModel;
    this.integrator = integrator;
    this.stateTime = stateTime;
  }

  protected abstract List<StateTime> move(Flow flow, Scalar period);

  @Override
  public final StateTime move(Tensor u, Scalar now) {
    List<StateTime> trajectory = move(StateSpaceModels.createFlow(stateSpaceModel, u), now.subtract(stateTime.time()));
    stateTime = trajectory.get(trajectory.size() - 1);
    return stateTime;
  }
}
