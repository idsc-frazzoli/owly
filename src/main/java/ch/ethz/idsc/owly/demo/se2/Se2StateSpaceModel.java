// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.owly.math.se2.Se2Integrator;
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
 * The Se2-StateSpaceModel has two control parameters:
 * 1) the angular rate
 * 2) the velocity
 * 
 * for forward-only motion simply disallow negative velocity values
 * 
 * since the se2 state space model is parameter free,
 * the access to the model is via a singleton instance
 * 
 * @see Se2Integrator */
public enum Se2StateSpaceModel implements StateSpaceModel {
  INSTANCE;
  // ---
  @Override
  public Tensor f(Tensor x, Tensor u) {
    // x = {px, py, theta}
    // u = {angular rate, speed}
    // speed: positive for forward motion, or negative for backward motion
    Scalar angle = x.Get(2);
    Scalar speed = u.Get(1);
    return Tensors.of( //
        Cos.FUNCTION.apply(angle), // change in px
        Sin.FUNCTION.apply(angle), // change in py
        u.Get(0) // change in angle
    ).multiply(speed); // overall scaling based on signed speed
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
