// code by jph
package ch.ethz.idsc.owl.math.r2;

import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Drop;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Round;

/** implementation is not consistent with Mathematica
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/CirclePoints.html">CirclePoints</a> */
public enum CirclePoints {
  ;
  /** the first coordinate is always {1, 0}.
   * the orientation of the points is counter-clockwise.
   * 
   * @param n
   * @return n x 2 matrix with evenly spaced points on the unit-circle */
  public static Tensor of(int n) {
    Tensor polygon = Tensors.empty();
    for (Tensor theta : Drop.tail(Subdivide.of(RealScalar.ZERO, Degree.of(360), n), 1))
      polygon.append(AngleVector.of(theta.Get()));
    return polygon;
  }

  public static Tensor elliptic(int n, Scalar width, Scalar height) {
    Tensor scale = Tensors.of(width, height);
    return Tensor.of(of(n).stream().map(row -> row.pmul(scale)));
  }

  public static Scalar roundToInteger(Scalar scalar) {
    Scalar round = Round.FUNCTION.apply(scalar);
    return Chop._12.close(round, scalar) ? round : scalar;
  }
}
