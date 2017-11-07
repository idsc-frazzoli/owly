// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.util.function.Function;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** the term "family" conveys the meaning that the translation
 * depends on a single parameter, for instance time */
public final class Se2Family implements BijectionFamily {
  private final Function<Scalar, Tensor> function;

  public Se2Family(Function<Scalar, Tensor> translation) {
    this.function = translation;
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
