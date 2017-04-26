// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class Se2StateSpaceModel implements StateSpaceModel {
  @Override
  public Tensor createFlow(Tensor x, Tensor u) { // u.length() == 2
    Scalar angle = x.Get(2);
    return Tensors.of(Cos.of(angle), Sin.of(angle), u.Get(0));
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
