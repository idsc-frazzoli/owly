// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** f(x,u)=u */
public abstract class SingleIntegrator extends EulerIntegrator {
  @Override
  public final Tensor flow(Tensor x, Tensor u) {
    return u;
  }

  @Override
  public final Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
