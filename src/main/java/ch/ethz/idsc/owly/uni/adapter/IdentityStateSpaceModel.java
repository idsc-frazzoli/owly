// code by bapaden and jph
package ch.ethz.idsc.owly.uni.adapter;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

// formerly "SingleIntegrator"
public final class IdentityStateSpaceModel implements StateSpaceModel {
  /** f(x,u) == u */
  @Override
  public Tensor createFlow(Tensor x, Tensor u) {
    return u;
  }

  @Override
  public Scalar getLipschitz() {
    return ZeroScalar.get();
  }
}
