// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** the term "family" conveys the meaning that the translation
 * depends on a single parameter, for instance time */
public final class TranslationFamily implements BijectionFamily {
  private final ScalarTensorFunction function;

  /** @param function maps a scalar to a vector in R^n */
  public TranslationFamily(ScalarTensorFunction function) {
    this.function = function;
  }

  @Override
  public TensorUnaryOperator forward(Scalar scalar) {
    Tensor offset = function.apply(scalar);
    return tensor -> tensor.add(offset);
  }

  @Override
  public TensorUnaryOperator inverse(Scalar scalar) {
    Tensor offset = function.apply(scalar);
    return tensor -> tensor.subtract(offset);
  }
}
