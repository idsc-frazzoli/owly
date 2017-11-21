// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.qty.Unit;
import ch.ethz.idsc.tensor.qty.UnitSystem;

/** inspired by
 * <a href="https://reference.wolfram.com/language/ref/Degree.html">Degree</a> */
public enum Degree {
  ;
  private static final Unit DEGREE = Unit.of("deg");

  /** the equivalent conversion formula in Mathematica is
   * QuantityMagnitude[Quantity[1, "Degrees"], "Radians"]
   * 
   * @param degree
   * @return radians == degree * PI / 180 */
  public static final Scalar of(Number degree) {
    return UnitSystem.SI().apply(Quantity.of(degree, DEGREE));
  }
}
