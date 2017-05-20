// code by jph
// adapted from document by Tobias Ewald
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.red.Norm;

public class SignedCurvature2D {
  /** Cross[{x, y}] == {-y, x}
   * 
   * @param vector
   * @return */
  // TODO extract
  public static Tensor cross2d(Tensor vector) {
    return Tensors.of(vector.Get(1).negate(), vector.Get(0));
  }

  /** @param a
   * @param b
   * @param c
   * @return inverse of radius of circle that interpolates the given points a, b, c */
  public static Scalar of(Tensor a, Tensor b, Tensor c) {
    Scalar v = b.subtract(a).dot(cross2d(c.subtract(b))).Get();
    Scalar w = b.subtract(a).dot(c.subtract(a)).Get();
    Scalar n = Norm._2.of(c.subtract(b));
    Scalar den = Hypot.bifunction.apply(v, w).multiply(n);
    return RealScalar.of(2).multiply(v).divide(den);
  }
}
