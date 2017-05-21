// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Reverse;
import ch.ethz.idsc.tensor.alg.TensorMap;

public enum Images {
  ;
  /** @param tensor
   * @return */
  public static Tensor displayOrientation(Tensor tensor) {
    return TensorMap.of(Reverse::of, tensor, 1);
  }
}
