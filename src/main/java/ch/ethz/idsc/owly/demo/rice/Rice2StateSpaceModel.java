// code by jph
package ch.ethz.idsc.owly.demo.rice;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Join;
import ch.ethz.idsc.tensor.red.Norm;

/** Double Integrator with friction
 * 
 * implementation for n-dimensional position and velocity */
public class Rice2StateSpaceModel implements StateSpaceModel {
  private final Scalar lambda;

  /** @param lambda strictly positive friction coefficient */
  public Rice2StateSpaceModel(Scalar lambda) {
    if (Scalars.lessEquals(lambda, RealScalar.ZERO))
      throw TensorRuntimeException.of(lambda);
    this.lambda = lambda;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    Tensor v = x.extract(u.length(), x.length());
    return Join.of(v, u.subtract(v).multiply(lambda));
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    // theory tells that:
    // lipschitz const is 2-norm of 4x4 state space matrix
    // 1 0 0 0
    // 0 1 0 0
    // 0 0 L 0
    // 0 0 0 L
    // where L == lambda
    // confirmed with mathematica
    return Norm._2.of(Tensors.of(RealScalar.ONE, lambda));
  }
}
