// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

class Se2Controls extends Controls {
  public Se2Controls(int num) {
    StateSpaceModel stateSpaceModel = new Se2StateSpaceModel();
    for (Tensor angle : Subdivide.of(RealScalar.of(-1), RealScalar.of(1), num)) {
      Tensor u = Tensors.of(angle);
      add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
  }
}
