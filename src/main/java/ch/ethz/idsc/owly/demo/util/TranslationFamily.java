// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;

/** the term "family" conveys the meaning that the translation
 * depends on a single parameter, for instance time */
public final class TranslationFamily implements RigidFamily {
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

  @Override
  public Tensor forward_se2(Scalar scalar) {
    Tensor offset = function.apply(scalar);
    Tensor matrix = IdentityMatrix.of(3);
    matrix.set(offset.Get(0), 0, 2);
    matrix.set(offset.Get(1), 1, 2);
    return matrix;
  }
}
