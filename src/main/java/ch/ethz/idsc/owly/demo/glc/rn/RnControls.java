// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;

public class RnControls {
  public static Tensor createR2RadialControls(final int num) {
    Tensor tensor = Tensors.empty();
    final double deltaAngle = 2 * Math.PI / num;
    for (int index = 0; index < num; ++index) {
      double angle = deltaAngle * index;
      Tensor u = Tensors.vector(Math.cos(angle), Math.sin(angle));
      tensor.append(u.unmodifiable());
    }
    return Chop.of(tensor);
  }
}
