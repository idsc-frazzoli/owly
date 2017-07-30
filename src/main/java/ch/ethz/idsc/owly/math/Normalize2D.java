// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.sca.Chop;

public enum Normalize2D {
  ;
  /** function normalizes a vector {x, y} unless the given vector
   * is {0, 0} in which case the return value is {0, 0}
   * 
   * @param x
   * @param y
   * @return {x, y} / |{x, y}|, or {0, 0} */
  public static Tensor unlessZero(Scalar x, Scalar y) {
    Scalar norm = Hypot.of(x, y);
    if (Scalars.isZero(norm))
      return Tensors.vector(0, 0);
    while (!Chop._15.close(norm, RealScalar.ONE)) {
      x = x.divide(norm);
      y = y.divide(norm);
      norm = Hypot.of(x, y);
    }
    return Tensors.of(x, y);
  }
}
