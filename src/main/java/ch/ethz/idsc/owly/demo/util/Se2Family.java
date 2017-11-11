// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.ScalarTensorFunction;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.RotationMatrix;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.ScalarUnaryOperator;
import ch.ethz.idsc.tensor.sca.Sin;

/** the term "family" conveys the meaning that the rigid transformation
 * depends on a single parameter, for instance time */
public class Se2Family implements RigidFamily {
  /** @param center
   * @param rotation
   * @return */
  public static RigidFamily rotationAround(Tensor center, ScalarUnaryOperator rotation) {
    return new Se2Family(time -> {
      Scalar theta = rotation.apply(time);
      return center.subtract(RotationMatrix.of(theta).dot(center)).append(theta);
    });
  }

  // ---
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

  @Override
  public Tensor forward_se2(Scalar scalar) {
    Tensor xya = function.apply(scalar); // {px, py, angle}
    Scalar angle = xya.Get(2);
    Scalar cos = Cos.FUNCTION.apply(angle);
    Scalar sin = Sin.FUNCTION.apply(angle);
    return Tensors.of( //
        Tensors.of(cos, sin.negate(), xya.Get(0)), //
        Tensors.of(sin, cos, xya.Get(1)), //
        Tensors.vector(0, 0, 1));
  }
}
