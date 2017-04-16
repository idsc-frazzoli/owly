// code by jph
package ch.ethz.idsc.owly.demo.glc.rice;

import ch.ethz.idsc.owly.adapter.RiceStateSpaceModel;
import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.owly.util.StateSpaceModels;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;

enum RiceControls {
  ;
  public static Controls createControls(int num) {
    StateSpaceModel stateSpaceModel = new RiceStateSpaceModel(RealScalar.of(.5));
    Controls controls = new Controls();
    for (Tensor u : Subdivide.of(DoubleScalar.of(-1), DoubleScalar.of(1), num))
      controls.add(StateSpaceModels.createFlow(stateSpaceModel, Tensors.of(u)));
    return controls;
  }
}
