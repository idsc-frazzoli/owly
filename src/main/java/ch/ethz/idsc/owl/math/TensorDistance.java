// code by jph
package ch.ethz.idsc.owl.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/**
 * 
 */
public interface TensorDistance {
  /** @param tensor
   * @return non-negative */
  Scalar distance(Tensor tensor);
}
