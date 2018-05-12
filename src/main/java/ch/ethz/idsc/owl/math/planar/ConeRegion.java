// code by jph
package ch.ethz.idsc.owl.math.planar;

import ch.ethz.idsc.owl.math.map.Se2Bijection;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Clip;
import ch.ethz.idsc.tensor.sca.Sign;

/** planar infinite cone region */
public class ConeRegion implements Region<Tensor> {
  private static final Scalar PI_HALF = RealScalar.of(Math.PI / 2);
  // ---
  private final TensorUnaryOperator inverse;
  private final Clip clip;
  private final Scalar semi;
  private final Scalar semi_pi_half;
  private final Tensor normal;

  /** @param xya vector of the form {x,y,angle} where {x,y} is the tip of the cone
   * and angle aligns with the center line of the cone
   * @param semi half angular width of cone in the interval [0, pi/2] */
  public ConeRegion(Tensor xya, Scalar semi) {
    inverse = new Se2Bijection(xya).inverse();
    clip = Clip.function(semi.negate(), semi);
    this.semi = Sign.requirePositiveOrZero(semi);
    semi_pi_half = semi.add(PI_HALF);
    normal = AngleVector.of(semi_pi_half);
    Sign.requirePositiveOrZero(normal.Get(1));
  }

  @Override // from Region<Tensor>
  public boolean isMember(Tensor tensor) {
    Tensor local = inverse.apply(tensor);
    return clip.isInside(ArcTan.of(local.Get(0), local.Get(1)));
  }

  public Scalar distance(Tensor tensor) {
    Tensor local = inverse.apply(tensor);
    local.set(Scalar::abs, 1); // normalize y coordinate
    Scalar angle = ArcTan.of(local.Get(0), local.Get(1)); // non-negative
    if (Scalars.lessThan(angle, semi))
      return RealScalar.ZERO;
    if (Scalars.lessThan(semi_pi_half, angle))
      return Norm._2.ofVector(local);
    return local.dot(normal).Get();
  }
}
