// code by jph
package ch.ethz.idsc.owly.math.car;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.red.Times;
import ch.ethz.idsc.tensor.sca.Tan;

/** class determines the no-slip velocity for tires at an offset from the center axis */
public class DifferentialSpeed {
  private final Scalar factor;

  /** @param x_front distance from rear to front axis
   * @param y_offset distance from center of rear axis to tire */
  public DifferentialSpeed(Scalar x_front, Scalar y_offset) {
    if (Scalars.isZero(x_front))
      throw TensorRuntimeException.of(x_front);
    this.factor = y_offset.divide(x_front);
  }

  /** @param v speed of vehicle at center of rear axis
   * @param beta turn angle at center of front axis
   * @return speed at y_offset from center of rear axis */
  public Scalar get(Scalar v, Scalar beta) {
    // v (L - r Tan[\[Beta]]) == L s
    return v.subtract(Times.of(factor, v, Tan.FUNCTION.apply(beta)));
  }
}
