// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.qty.UnitSystem;

public enum RotationUtils {
  ;
  /** the equivalent conversion formula in Mathematica is
   * QuantityMagnitude[Quantity[1, "Degrees"], "Radians"]
   * 
   * @param degree
   * @return radians == degree * PI / 180 */
  public static final Scalar DEGREE(Number degree) {
    return UnitSystem.SI().apply(Quantity.of(degree, "deg"));
  }
}
