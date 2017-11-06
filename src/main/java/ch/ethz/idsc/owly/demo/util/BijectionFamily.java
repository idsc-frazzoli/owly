// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;

public interface BijectionFamily {
  TensorUnaryOperator forward(Scalar scalar);

  TensorUnaryOperator inverse(Scalar scalar);
}
