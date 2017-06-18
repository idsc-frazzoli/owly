// code by jph
package ch.ethz.idsc.owly.demo.se2r;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

class Se2rStateSpaceModel implements StateSpaceModel {
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // u.Get(0) contains angle
    // u.Get(1) should be 1 for forward motion or -1 for backward motion
    Scalar angle = x.Get(2);
    Tensor tangent = Tensors.of(Cos.of(angle), Sin.of(angle), u.Get(0));
    return tangent.multiply(u.Get(1));
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
