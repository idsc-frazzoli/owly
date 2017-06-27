// code by jph
package ch.ethz.idsc.owly.demo.lv;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

public class LvControls {
  public static Collection<Flow> set(Scalar f0, Scalar f1) {
    StateSpaceModel stateSpaceModel = new LvStateSpaceModel(f0, f1);
    List<Flow> list = new ArrayList<>();
    for (Tensor u : Subdivide.of(0, 1, 1))
      list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(u)));
    // list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.vector(0)));
    return list;
  }
}
