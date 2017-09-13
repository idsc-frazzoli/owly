// code by edo
package ch.ethz.idsc.owly.demo.drift;

import java.util.Collection;
import java.util.HashSet;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

public enum DriftControls {
  ;
  public static Collection<Flow> create() {
    return _create(new DriftStateSpaceModel(new DriftParameters()));
  }

  public static Collection<Flow> createExtended() {
    return _create(new DriftExtStateSpaceModel(new DriftParameters()));
  }

  private static Collection<Flow> _create(StateSpaceModel stateSpaceModel) {
    Collection<Flow> collection = new HashSet<>();
    for (Tensor theta : Subdivide.of(-20 * Math.PI / 180, 20 * Math.PI / 180, 10)) {
      Tensor u = Tensors.of(theta, RealScalar.of(1815));
      collection.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
    return collection;
  }
}
