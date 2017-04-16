// code by jph
package ch.ethz.idsc.owly.adapter;

import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class RiceStateSpaceModel implements StateSpaceModel {
  private final Scalar lambda;

  public RiceStateSpaceModel(Scalar lambda) {
    this.lambda = lambda;
  }

  @Override
  public Tensor flow(Tensor x, Tensor u) {
    Scalar v = x.Get(1);
    return Tensors.of( //
        v, // x0' = v // v = x1
        lambda.multiply(u.Get(0).subtract(v)) // x1' = lambda * (u - v)
    );
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return lambda.invert(); // TODO check, or Sqrt(1+lambda^2) ?
  }
}
