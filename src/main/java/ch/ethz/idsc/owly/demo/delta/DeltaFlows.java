// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.ArrayList;
import java.util.Collection;

import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.owl.math.StateSpaceModels;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.planar.CirclePoints;
import ch.ethz.idsc.owly.demo.util.FlowsInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

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
}
