// code by jph
package ch.ethz.idsc.owly.demo.lv;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

public enum LvControls {
  ;
  public static Collection<Flow> create(StateSpaceModel stateSpaceModel, int num) {
    List<Flow> list = new ArrayList<>();
    for (Tensor u : Subdivide.of(0, 1, num))
      list.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(u)));
    return list;
  }
}
