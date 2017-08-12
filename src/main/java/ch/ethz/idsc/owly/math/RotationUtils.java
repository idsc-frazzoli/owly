// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;

public enum RotationUtils {
  ;
  /** @param degree
   * @return radians */
  public static final Scalar DEGREE(int degree) {
    return RationalScalar.of(degree, 180).multiply(DoubleScalar.of(Math.PI));
  }
}
