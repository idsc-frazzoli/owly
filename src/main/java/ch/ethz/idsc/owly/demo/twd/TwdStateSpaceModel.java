// code by jph
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** two wheel drive (non-holonomic wheeled mobile robot?)
 * 
 * robot with one axle
 * center of axle at (px, py)
 * 2 wheels (left, right) with separate controlled speed (wl, wr) */
public class TwdStateSpaceModel implements StateSpaceModel {
  // TODO JONAS falls Dein model von Parametern abhaengt, dann definiere einen constructor.
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // TODO JONAS implement
    // state contains (px, py, theta) == position of axis center, theta is orientation
    // u contains (wl, wr) == speed of left wheel and speed of right wheel
    throw new RuntimeException();
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
