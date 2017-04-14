// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;

class RnControls {
  public static Tensor createR2RadialControls(final int num) {
    Tensor tensor = Tensors.empty();
    final double deltaAngle = 2 * Math.PI / num;
    for (int index = 0; index < num; ++index) {
      double angle = deltaAngle * index;
      tensor.append(Tensors.vector(Math.cos(angle), Math.sin(angle)));
    }
    return Chop.of(tensor);
  }
}
