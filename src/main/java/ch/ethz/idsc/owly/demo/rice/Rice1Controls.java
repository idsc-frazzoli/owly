// code by jph
package ch.ethz.idsc.owly.demo.rice;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

public enum Rice1Controls {
  ;
  public static Collection<Flow> createControls(Scalar lambda, int num) {
    // FIXME JAN top priority
    StateSpaceModel stateSpaceModel = new Rice2StateSpaceModel(lambda);
    List<Flow> list = new ArrayList<>();
    for (Tensor u : Subdivide.of(DoubleScalar.of(-1), DoubleScalar.of(1), num))
      list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(u)));
    return list;
  }
}
