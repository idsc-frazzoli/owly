// code by jph
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Times;
import ch.ethz.idsc.tensor.sca.Abs;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

/** two wheel drive (non-holonomic wheeled mobile robot?)
 * 
 * robot with one axle
 * center of axle at (px, py)
 * 2 wheels (left, right) with separate controlled speed (wl, wr) [rad/s]
 * Theory from: http://planning.cs.uiuc.edu/node659.html */
public class TwdStateSpaceModel implements StateSpaceModel {
  private final Scalar wheelRadius; // R
  private final Scalar wheelDistance; // L
  private final Tensor wheelspeeds_max;

  public TwdStateSpaceModel(Scalar wheelRadius, Scalar wheelDistance, Tensor wheelspeed_max) {
    this.wheelRadius = wheelRadius;
    this.wheelDistance = wheelDistance;
    this.wheelspeeds_max = wheelspeed_max;
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
    // TODO JONAS implement
    // state contains (px, py, theta) == position of axis center, theta is orientation
    // u contains (wl, wr) == speed of left wheel and speed of right wheel
  }

  public Tensor getWheelspeeds_max() {
    return wheelspeeds_max;
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Tensor getLipschitz() {
    // TODO JONAS should be:
    // max((wl+wr))*wheelRadius*0.5 for horizontal velocities
    Scalar horizontalLipschitz = Times.of((getWheelspeeds_max().Get(0).add(getWheelspeeds_max()).Get(1)), wheelRadius, RealScalar.of(0.5));
    // wheelRadius/wheelDistance *max(abs(wl),abs(wr)) for rotational
    Scalar rotationalLipschitz = Max.of(Abs.of(getWheelspeeds_max().Get(0)), Abs.of(getWheelspeeds_max().Get(1)));
    return Tensors.of(horizontalLipschitz, horizontalLipschitz, rotationalLipschitz);
  }
}
