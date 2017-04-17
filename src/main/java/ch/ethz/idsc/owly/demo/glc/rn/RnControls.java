// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.glc.core.Controls;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.uni.adapter.IdentityStateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;

enum RnControls {
  ;
  public static Controls createR2RadialControls(final int num, Scalar amp) {
    StateSpaceModel stateSpaceModel = new IdentityStateSpaceModel();
    Controls controls = new Controls();
    // TODO use scalars
    final double deltaAngle = 2 * Math.PI / num;
    for (int index = 0; index < num; ++index) {
      double angle = deltaAngle * index;
      Tensor u = Chop.of(Tensors.vector(Math.cos(angle), Math.sin(angle)).multiply(amp));
      controls.add(StateSpaceModels.createFlow(stateSpaceModel, u));
    }
    return controls;
  }
}
