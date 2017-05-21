// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public enum Cross2D {
  ;
  /** Cross[{x, y}] == {-y, x}
   * 
   * @param vector
   * @return */
  public static Tensor of(Tensor vector) {
    return Tensors.of(vector.Get(1).negate(), vector.Get(0));
  }
}
