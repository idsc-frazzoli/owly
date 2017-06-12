// code by bapaden and jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

// formerly "SingleIntegrator"
public enum IdentityStateSpaceModel implements StateSpaceModel {
  INSTANCE;
  // ---
  /** f(x,u) == u */
  @Override
  public Tensor f(Tensor x, Tensor u) {
    return u;
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ZERO; // TODO why doesn't max speed matter here!?
  }
}
