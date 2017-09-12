// code by edo
package ch.ethz.idsc.owly.demo.drift;

import java.util.Collection;
import java.util.HashSet;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

public enum DriftControls {
  ;
  public static Collection<Flow> create() {
    Collection<Flow> collection = new HashSet<>();
    DriftParameters driftParameters = new DriftParameters();
    DriftStateSpaceModel driftStateSpaceModel = new DriftStateSpaceModel(driftParameters);
    for (Tensor theta : Subdivide.of(-20*Math.PI/180, 20*Math.PI/180, 10)) {
      Tensor u = Tensors.of(theta, RealScalar.of(1815));
      collection.add(StateSpaceModels.createFlow(driftStateSpaceModel, u));
    }
    return collection;
  }
}
