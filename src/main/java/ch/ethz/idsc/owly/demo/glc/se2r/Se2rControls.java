// code by jph
package ch.ethz.idsc.owly.demo.glc.se2r;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

class Se2rControls {
  public static Collection<Flow> createControls(Scalar angle_max, int num) {
    StateSpaceModel stateSpaceModel = new Se2rStateSpaceModel();
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(angle_max.negate(), angle_max, num)) {
      {
        Tensor u = Tensors.of(angle, RealScalar.ONE);
        list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
      }
      {
        Tensor u = Tensors.of(angle, RealScalar.ONE.negate());
        list.add(StateSpaceModels.createFlow(stateSpaceModel, u));
      }
    }
    return list;
  }
}
