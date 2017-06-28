// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.red.Hypot;

// TODO find a better name and then relocate
public enum StablePytagoras {
  ;
  /** @param x
   * @param y
   * @return x / (x^2 + y^2) or 0 for (x,y) == (0,0) */
  public static Scalar of(Scalar x, Scalar y) {
    Scalar den = Hypot.bifunction.apply(x, y);
    if (Scalars.isZero(den))
      return den.zero();
    if (Scalars.lessThan(den, x.abs()))
      throw TensorRuntimeException.of(x, y, den);
    return x.divide(den);
  }
}
