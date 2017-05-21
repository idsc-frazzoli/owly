// code by jph
package ch.ethz.idsc.owly.demo.glc.rice1;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;

class Rice1StateSpaceModel implements StateSpaceModel {
  private final Scalar lambda;

  public Rice1StateSpaceModel(Scalar lambda) {
    this.lambda = lambda;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    Scalar v = x.Get(1);
    return Tensors.of( //
        v, // x0' = v // v = x1
        lambda.multiply(u.Get(0).subtract(v)) // x1' = lambda * (u - v)
    );
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return Norm._2.of(Tensors.of(RealScalar.ONE, lambda));
  }
}
