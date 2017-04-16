// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.Tensor;

public interface Region {
  /** @param tensor
   * @return true if tensor is member/part of/inside region */
  boolean isMember(Tensor tensor);
}
