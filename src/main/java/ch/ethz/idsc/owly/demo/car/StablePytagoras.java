// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;

// TODO find a better name and then relocate
public enum StablePytagoras {
  ;
  /** function normalizes a vector {x, y} unless the given vector
   * is {0, 0} in which case the return value is {0, 0}
   * 
   * @param x
   * @param y
   * @return */
  public static Tensor normalize(Scalar x, Scalar y) {
    Scalar den = Hypot.bifunction.apply(x, y);
    if (Scalars.isZero(den))
      return Tensors.vector(0, 0);
    return Tensors.of(x, y).multiply(den.invert());
  }

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
