// code by bapaden and jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** implementation for arbitrary dimensions
 * 
 * formerly "IdentityStateSpaceModel" */
public enum SingleIntegratorStateSpaceModel implements StateSpaceModel {
  INSTANCE;
  // ---
  /** f(x,u) == u */
  @Override
  public Tensor f(Tensor x, Tensor u) {
    return u;
  }

  @Override
  public Scalar getLipschitz() {
    // | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 |
    // | f(x_1, u) - f(x_2, u) | == | u - u | == 0
    // therefore L == 0
    return RealScalar.ZERO;
  }
}
