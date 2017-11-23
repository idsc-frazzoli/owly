// code by jph
package ch.ethz.idsc.owly.demo.rice;

import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.alg.Join;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.sca.Sign;

/** Single Integrator with friction
 * 
 * implementation for n-dimensional velocity */
public class Duncan2StateSpaceModel implements StateSpaceModel {
  private final Scalar lambda;

  /** @param lambda non-negative friction coefficient typically with unit [s^-1] */
  public Duncan2StateSpaceModel(Scalar lambda) {
    if (Sign.isNegative(lambda))
      throw TensorRuntimeException.of(lambda);
    this.lambda = lambda;
  }

  @Override // from StateSpaceModel
  public Tensor f(Tensor x, Tensor u) {
    Tensor v = x.extract(u.length(), x.length());
    return Join.of(v, u.subtract(v.multiply(lambda)));
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    // how about when lambda is Quantity?
    return Hypot.BIFUNCTION.apply(RealScalar.ONE, lambda);
  }
}
