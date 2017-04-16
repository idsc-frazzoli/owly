// code by jph
package ch.ethz.idsc.owly.demo.glc.psu;

import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sin;

class PsuIntegrator implements StateSpaceModel {
  @Override
  public Tensor flow(Tensor x, Tensor u) {
    // equation (10)
    // x0' = x1
    // x1' = -sin(x0) + u
    return Tensors.of(x.Get(1), u.Get(0).subtract(Sin.function.apply(x.Get(0))));
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.of(2); // TODO check
  }
}
