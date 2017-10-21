// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.qty.UnitSystem;

public enum RotationUtils {
  ;
  /** @param degree
   * @return radians */
  public static final Scalar DEGREE(Number degree) {
    return UnitSystem.SI().apply(Quantity.of(degree, "deg"));
  }
}
