// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Tensor;

public interface SlipInterface {
  /** @return vector with 2 entries that measure the degree of friction (in tire coordinates) */
  Tensor slip();
}
