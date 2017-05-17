// code by jph
package ch.ethz.idsc.owly.demo.glc.twd;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** two wheel drive
 * non-holonomic wheeled mobile robot */
public class TwdStateSpaceModel implements StateSpaceModel {
  @Override
  public Tensor createFlow(Tensor x, Tensor u) {
    // u only contains angle, single elements
    // FIXME
    throw new RuntimeException();
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
