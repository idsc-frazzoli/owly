// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Join;
import ch.ethz.idsc.tensor.red.Norm;

class Rice2StateSpaceModel implements StateSpaceModel {
  private final Scalar lambda;

  public Rice2StateSpaceModel(Scalar lambda) {
    this.lambda = lambda;
  }

  @Override
  public Tensor createFlow(Tensor x, Tensor u) { // u.length() == 2
    Tensor v = x.extract(2, 4);
    return Join.of(v, u.subtract(v).multiply(lambda));
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return Norm._2.of(Tensors.of(RealScalar.ONE, lambda)); // confirmed with mathematica
  }
}
