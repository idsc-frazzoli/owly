// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.demo.se2r.Se2rStateSpaceModel;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** Nonholonomic Wheeled Robot
 * 
 * bapaden phd thesis: (5.5.12)
 * 
 * The Se2-StateSpaceModel has a single control parameter: the angle
 * In order to control angle and speed use {@link Se2rStateSpaceModel}. */
public enum Se2StateSpaceModel implements StateSpaceModel {
  INSTANCE;
  // ---
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // u only contains angle, u.length() == 1
    Scalar angle = x.Get(2);
    return Tensors.of(Cos.of(angle), Sin.of(angle), u.Get(0));
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
