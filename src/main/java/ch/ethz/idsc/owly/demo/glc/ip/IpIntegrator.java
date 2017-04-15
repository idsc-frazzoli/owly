// code by jph
package ch.ethz.idsc.owly.demo.glc.ip;

import ch.ethz.idsc.owly.glc.adapter.EulerIntegrator;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

class IpIntegrator extends EulerIntegrator {
  @Override
  public Tensor flow(Tensor x, Tensor u) {
    // TODO
    return null;
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }

  @Override
  public Scalar getMaxTimeStep() {
    return RealScalar.ONE;
  }
}
