// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;

class Rice2StateSpaceModel implements StateSpaceModel {
  private final Scalar lambda;

  public Rice2StateSpaceModel(Scalar lambda) {
    this.lambda = lambda;
  }

  @Override
  public Tensor flow(Tensor x, Tensor u) { // u.length() == 2
    Scalar v0 = x.Get(2);
    Scalar v1 = x.Get(3);
    return Tensors.of( //
        v0, // x0' = v // v = x1
        v1, //
        lambda.multiply(u.Get(0).subtract(v0)), // x1' = lambda * (u - v)
        lambda.multiply(u.Get(1).subtract(v1)) //
    );
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return Norm._2.of(Tensors.of(RealScalar.ONE, lambda)); // confirmed with mathematica
  }
}
