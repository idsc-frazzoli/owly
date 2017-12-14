// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.ArrayList;
import java.util.Collection;

import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owly.demo.util.FlowsInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.CirclePoints;

/*
 * Same as DeltaFlows controls, but with added Zero Flow (Drifting in the Water)
 */
public class DeltaFlowsDrifting implements FlowsInterface {
  private final StateSpaceModel stateSpaceModel;
  private final Scalar amp;

  public DeltaFlowsDrifting(StateSpaceModel stateSpaceModel, Scalar amp) {
    this.stateSpaceModel = stateSpaceModel;
    this.amp = amp;
  }

  @Override
  public Collection<Flow> getFlows(int resolution) {
    Collection<Flow> collection = new ArrayList<>();
    collection.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(0, 0)));
    for (Tensor u : CirclePoints.of(resolution - 1))
      collection.add(StateSpaceModels.createFlow(stateSpaceModel, u.multiply(amp)));
    return collection;
  }
}
