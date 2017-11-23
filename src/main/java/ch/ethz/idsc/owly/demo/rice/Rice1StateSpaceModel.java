// code by jph
package ch.ethz.idsc.owly.demo.rice;

import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.sca.Exp;
import ch.ethz.idsc.tensor.sca.Sign;

/** Important:
 * The use of {@link Duncan1StateSpaceModel} is preferred and
 * supports the use of units.
 * 
 * <p>Rice1StateSpaceModel is a single Integrator with friction.
 * Rice1StateSpaceModel is unit less.
 * The implementation for n-dimensional velocity */
public class Rice1StateSpaceModel implements StateSpaceModel {
  public static StateSpaceModel of(Scalar mu) {
    return new Rice1StateSpaceModel(Exp.of(mu));
  }

  // ---
  private final Scalar lambda;

  /** @param lambda strictly positive friction coefficient */
  private Rice1StateSpaceModel(Scalar lambda) {
    if (Sign.isNegativeOrZero(lambda))
      throw TensorRuntimeException.of(lambda);
    this.lambda = lambda;
  }

  @Override // from StateSpaceModel
  public Tensor f(Tensor x, Tensor u) {
    Tensor v = x;
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
