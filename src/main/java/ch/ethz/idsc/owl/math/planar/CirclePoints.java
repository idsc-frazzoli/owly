// code by jph
package ch.ethz.idsc.owl.math.planar;

import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Range;
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
    Tensor angles = Range.of(0, n).multiply(Degree.of(RationalScalar.of(360, n)));
    return Tensor.of(angles.stream() //
        .map(Scalar.class::cast) //
        .map(AngleVector::of));
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
