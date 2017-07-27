// code by jl
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** two wheel drive (non-holonomic wheeled mobile robot?)
 * 
 * robot with one axle
 * center of axle at (px, py)
 * 2 wheels (left, right) with separate controlled speed (wl, wr) [rad/s]
 * max (wl) = max (wr) = 1
 * Theory from: http://planning.cs.uiuc.edu/node659.html */
@SuppressWarnings("serial")
public class TwdStateSpaceModel implements StateSpaceModel {
  private final Scalar wheelRadius; // R
  private final Scalar wheelDistance; // L

  // TODO JONAS: set wheelspeed_max to 1
  // wheelspeed_max = angular wheel velocity
  public TwdStateSpaceModel(Scalar wheelRadius, Scalar wheelDistance) {
    this.wheelRadius = wheelRadius;
    this.wheelDistance = wheelDistance;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    // (wl +wr)/2
    Scalar translateSpeed = u.Get(0).add(u.Get(1)).divide(RealScalar.of(2));
    // r/L*(wr-wl)
    Scalar rotationalSpeed = u.Get(1).subtract(u.Get(0)).multiply(wheelRadius).divide(wheelDistance);
    // r*translateSpeed * cos(theta)
    Scalar xDot = wheelRadius.multiply(translateSpeed).multiply(Cos.of(x.Get(2)));
    // r*translateSpeed * sin(theta)
    Scalar yDot = wheelRadius.multiply(translateSpeed).multiply(Sin.of(x.Get(2)));
    return Tensors.of(xDot, yDot, rotationalSpeed);
    // state contains (px, py, theta) == position of axis center, theta is orientation
    // u contains (wl, wr) == speed of left wheel and speed of right wheel
  }

  /** Returns the Lipschitzconstant of the TWDmodel.
   * /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 |
   * f = r/2*(Ul+Ur)*cos(theta) r/2(Ul*Ur)*sin(theta) r/L(UR-Ul)
   * L = max (eig(gradient(f))
   * with mathematica:
   * L = 1/2|r|*|Ul+Ur|
   * Assumption: max(rotational_wheelspeed) = max (Ul) = max(Ur) = 1
   * L = r */
  public Scalar getLipschitz() {
    return wheelRadius;
  }
}
