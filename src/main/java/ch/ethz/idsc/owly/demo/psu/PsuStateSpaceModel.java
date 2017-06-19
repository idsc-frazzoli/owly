// code by jph
package ch.ethz.idsc.owly.demo.psu;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Sin;

/** Pendulum Swing-up state space model
 * 
 * bapaden phd thesis: (5.5.4) */
public enum PsuStateSpaceModel implements StateSpaceModel {
  INSTANCE;
  // ---
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // equation (10)
    // x0' = x1
    // x1' = -sin(x0) + u
    return Tensors.of(x.Get(1), u.Get(0).subtract(Sin.of(x.Get(0))));
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
