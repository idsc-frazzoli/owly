// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.ArrayList;
import java.util.Collection;

import ch.ethz.idsc.owl.math.SingleIntegratorStateSpaceModel;
import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owly.demo.util.FlowsInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.lie.CirclePoints;

public class DeltaFlows implements FlowsInterface {
  private final StateSpaceModel stateSpaceModel;
  private final Scalar amp;

  public DeltaFlows(StateSpaceModel stateSpaceModel, Scalar amp) {
    this.stateSpaceModel = stateSpaceModel;
    this.amp = amp;
  }

  @Override
  public Collection<Flow> getFlows(int resolution) {
    Collection<Flow> collection = new ArrayList<>();
    for (Tensor u : CirclePoints.of(resolution))
      collection.add(StateSpaceModels.createFlow(stateSpaceModel, u.multiply(amp)));
    return collection;
  }

  public Flow stayPut() {
    StateSpaceModel stateSpaceModel = SingleIntegratorStateSpaceModel.INSTANCE;
    return StateSpaceModels.createFlow(stateSpaceModel, Array.zeros(2));
  }
}
