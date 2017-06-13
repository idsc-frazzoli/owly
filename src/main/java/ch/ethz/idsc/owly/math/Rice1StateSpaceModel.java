// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;

/** Single Integrator with friction
 * 
 * implementation for n-dimensional velocity */
public class Rice1StateSpaceModel implements StateSpaceModel {
  private final Scalar lambda;

  /** @param lambda strictly positive friction coefficient */
  public Rice1StateSpaceModel(Scalar lambda) {
    if (Scalars.lessEquals(lambda, RealScalar.ZERO))
      throw TensorRuntimeException.of(lambda);
    this.lambda = lambda;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    Tensor v = x; // == x.extract(0, u.length());
    return u.subtract(v).multiply(lambda);
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    // theory tells that:
    // lipschitz const is 2-norm of 2x2 state space matrix
    // L 0
    // 0 L
    // where L == lambda
    // confirmed with mathematica
    return lambda.abs();
  }
}
