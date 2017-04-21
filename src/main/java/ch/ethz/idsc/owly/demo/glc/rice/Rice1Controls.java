// code by jph
package ch.ethz.idsc.owly.demo.glc.rice;

import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

class Rice1Controls extends Controls {
  public Rice1Controls(int num) {
    StateSpaceModel stateSpaceModel = new Rice1StateSpaceModel(RealScalar.of(.5));
    for (Tensor u : Subdivide.of(DoubleScalar.of(-1), DoubleScalar.of(1), num))
      add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(u)));
  }
}
