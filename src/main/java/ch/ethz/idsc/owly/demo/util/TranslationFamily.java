// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public abstract class TranslationFamily implements BijectionFamily {
  @Override
  public final TensorUnaryOperator forward(Scalar scalar) {
    Tensor offset = translation(scalar);
    return tensor -> tensor.add(offset);
  }

  @Override
  public final TensorUnaryOperator inverse(Scalar scalar) {
    Tensor offset = translation(scalar);
    return tensor -> tensor.subtract(offset);
  }

  public abstract Tensor translation(Scalar scalar);
}
