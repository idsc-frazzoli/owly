// code by jph
package ch.ethz.idsc.owly.demo.util;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.lie.RotationMatrix;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.ScalarUnaryOperator;
import ch.ethz.idsc.tensor.sca.Sin;

/** the term "family" conveys the meaning that the rigid transformation
 * depends on a single parameter, for instance time */
public class So2Family implements RigidFamily {
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

  @Override
  public Tensor forward_se2(Scalar scalar) {
    Scalar angle = function.apply(scalar);
    Scalar cos = Cos.FUNCTION.apply(angle);
    Scalar sin = Sin.FUNCTION.apply(angle);
    return Tensors.of( //
        Tensors.of(cos, sin.negate(), RealScalar.ZERO), //
        Tensors.of(sin, cos, RealScalar.ZERO), //
        Tensors.vector(0, 0, 1));
  }
}
