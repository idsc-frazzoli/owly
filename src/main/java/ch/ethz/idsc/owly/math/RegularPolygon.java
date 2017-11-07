// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Drop;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.lie.AngleVector;

/** inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegularPolygon.html">RegularPolygon</a> */
public enum RegularPolygon {
  ;
  /** @param n
   * @return n x 2 matrix with evenly spaced points on the unit-circle */
  public static Tensor of(int n) {
    Tensor polygon = Tensors.empty();
    for (Tensor theta : Drop.tail(Subdivide.of(RealScalar.ZERO, RotationUtils.DEGREE(360), n), 1))
      polygon.append(AngleVector.of(theta.Get()));
    return polygon;
  }

  public static Tensor elliptic(int n, Scalar width, Scalar height) {
    Tensor scale = Tensors.of(width, height);
    return Tensor.of(of(n).stream().map(row -> row.pmul(scale)));
  }
}
