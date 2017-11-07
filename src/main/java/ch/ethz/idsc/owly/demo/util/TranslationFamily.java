// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.util.function.Function;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public final class TranslationFamily implements BijectionFamily {
  private final Function<Scalar, Tensor> translation;

  public TranslationFamily(Function<Scalar, Tensor> translation) {
    this.translation = translation;
  }

  @Override
  public TensorUnaryOperator forward(Scalar scalar) {
    Tensor offset = translation.apply(scalar);
    return tensor -> tensor.add(offset);
  }

  @Override
  public TensorUnaryOperator inverse(Scalar scalar) {
    Tensor offset = translation.apply(scalar);
    return tensor -> tensor.subtract(offset);
  }
}
