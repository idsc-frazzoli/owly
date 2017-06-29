// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;

public enum Normalize2D {
  ;
  /** function normalizes a vector {x, y} unless the given vector
   * is {0, 0} in which case the return value is {0, 0}
   * 
   * @param x
   * @param y
   * @return {x, y} / |{x, y}|, or {0, 0} */
  public static Tensor unlessZero(Scalar x, Scalar y) {
    Scalar norm = Hypot.bifunction.apply(x, y);
    return Scalars.isZero(norm) ? //
        Tensors.vector(0, 0) : Tensors.of(x, y).multiply(norm.invert());
  }
}
