// code by jph
package ch.ethz.idsc.owly.math.region;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Tensor;

/** function R^n -> {true, false} */
public interface Region extends Serializable {
  /** @param tensor
   * @return true if tensor is member/part of/inside region */
  boolean isMember(Tensor tensor);
}
