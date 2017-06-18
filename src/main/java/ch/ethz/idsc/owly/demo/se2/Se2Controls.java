// code by jph
package ch.ethz.idsc.owly.demo.se2;

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

public enum Se2Controls {
  ;
  // ---
  public static Collection<Flow> createControls(Scalar angle_max, int num) {
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(angle_max.negate(), angle_max, num)) {
      Tensor u = Tensors.of(angle);
      list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
    return list;
  }
}
