// code by jl
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.NumberQ;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Mean;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** two wheel drive (non-holonomic wheeled mobile robot)
 * 
 * robot with one axle
 * center of axle at (px, py)
 * 2 wheels (left, right) with separate controlled speed (wl, wr) [rad/s]
 * max (wl) = max (wr) = 1
 * Theory from: http://planning.cs.uiuc.edu/node659.html */
public class TwdStateSpaceModel implements StateSpaceModel {
  private final Scalar wheelRadius; // R
  private final Scalar wheelDistanceInverse; // L

  /** @param wheelRadius
   * @param wheelDistance */
  public TwdStateSpaceModel(Scalar wheelRadius, Scalar wheelDistance) {
    this.wheelRadius = wheelRadius;
    this.wheelDistanceInverse = wheelDistance.invert();
    GlobalAssert.that(NumberQ.of(wheelDistanceInverse));
  }

  /** implementation of eqs (13.16)
   * http://planning.cs.uiuc.edu/node659.html
   * 
   * ^ +y
   * |
   * WL u.Get(0)
   * --
   * |
   * |
   * |------> +x
   * |
   * |
   * --
   * WR u.Get(1)
   * 
   * state x contains (px, py, theta) == position of axis center, theta is orientation
   * u contains (wl, wr) == speed of left wheel and speed of right wheel */
  @Override
  public Tensor f(Tensor x, Tensor u) {
    Scalar mean_u = Mean.of(u).Get(); // (wl + wr) / 2
    Scalar theta = x.Get(2);
    Tensor unscaled = Tensors.of( //
        mean_u.multiply(Cos.of(theta)), // (wl + wr) / 2 * cos(theta)
        mean_u.multiply(Sin.of(theta)), //
        u.Get(1).subtract(u.Get(0)).multiply(wheelDistanceInverse)); // (wr - wl) / L
    return unscaled.multiply(wheelRadius); // scale everything by wheel radius
  }

  /** Returns the Lipschitzconstant of the TWDmodel.
   * /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 |
   * f = r/2*(Ul+Ur)*cos(theta) r/2(Ul*Ur)*sin(theta) r/L(UR-Ul)
   * L = max (eig(gradient(f))
   * with mathematica:
   * L = 1/2|r|*|Ul+Ur|
   * Assumption: max(rotational_wheelspeed) = max (Ul) = max(Ur) = 1
   * L = r */
  @Override
  public Scalar getLipschitz() {
    return wheelRadius;
  }
}
