// code by jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.tensor.Tensor;

/** time invariant differential constraint */
public interface Flow {
  /** @param x
   * @return tangent of flow evaluated at x */
  Tensor at(Tensor x);

  /** @return identifier/control that determines the flow */
  Tensor getU();
}
