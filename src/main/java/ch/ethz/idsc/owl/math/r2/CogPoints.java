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

public enum CogPoints {
  ;
  /** the first coordinate is always {1, 0}.
   * the orientation of the points is counter-clockwise.
   * 
   * @param n
   * @return 2 * n x 2 matrix */
  public static Tensor of(int n, Scalar s_hi, Scalar s_lo) {
    int n4 = n * 4;
    Tensor polygon = Tensors.empty();
    Scalar[] rad = new Scalar[] { s_hi, s_hi, s_lo, s_lo };
    int count = 0;
    for (Tensor theta : Drop.tail(Subdivide.of(RealScalar.ZERO, Degree.of(360), n4), 1)) {
      polygon.append(AngleVector.of(theta.Get()).multiply(rad[count++ % 4]));
    }
    return polygon;
  }
}
