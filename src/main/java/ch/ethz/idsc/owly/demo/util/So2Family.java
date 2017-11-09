// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.lie.RotationMatrix;
import ch.ethz.idsc.tensor.sca.ScalarUnaryOperator;

/** the term "family" conveys the meaning that the rigid transformation
 * depends on a single parameter, for instance time */
public final class So2Family implements BijectionFamily {
  private final ScalarUnaryOperator function;

  public So2Family(ScalarUnaryOperator function) {
    this.function = function;
  }

  @Override
  public TensorUnaryOperator forward(Scalar scalar) {
    Scalar angle = function.apply(scalar);
    Tensor matrix = RotationMatrix.of(angle);
    return tensor -> matrix.dot(tensor);
  }

  @Override
  public TensorUnaryOperator inverse(Scalar scalar) {
    Scalar angle = function.apply(scalar);
    Tensor matrix = RotationMatrix.of(angle.negate());
    return tensor -> matrix.dot(tensor);
  }
}
