// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.lie.RotationMatrix;

/** the term "family" conveys the meaning that the rigid transformation
 * depends on a single parameter, for instance time */
public final class Se2Family implements BijectionFamily {
  private final ScalarTensorFunction function;

  public Se2Family(ScalarTensorFunction function) {
    this.function = function;
  }

  @Override
  public TensorUnaryOperator forward(Scalar scalar) {
    Tensor xya = function.apply(scalar); // {px, py, angle}
    Tensor matrix = RotationMatrix.of(xya.Get(2));
    Tensor offset = xya.extract(0, 2);
    return tensor -> matrix.dot(tensor).add(offset);
  }

  @Override
  public TensorUnaryOperator inverse(Scalar scalar) {
    Tensor xya = function.apply(scalar); // {px, py, angle}
    Tensor matrix = RotationMatrix.of(xya.Get(2).negate());
    Tensor offset = xya.extract(0, 2);
    return tensor -> matrix.dot(tensor.subtract(offset));
  }
}
