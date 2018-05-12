// code by jph
package ch.ethz.idsc.owl.math.region;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.math.map.Se2Bijection;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.opt.TensorScalarFunction;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Sign;

/** distance to cone 2 region */
public class Cone2Distance implements TensorScalarFunction {
  // private static final Scalar BOUND = RealScalar.of(Math.PI / 2);
  // ---
  private final TensorUnaryOperator inverse;
  private final Scalar semi;
  private final Scalar semi_pi_half;
  private final Tensor normal;

  /** @param xya vector of the form {x,y,angle} where {x,y} is the tip of the cone
   * and angle aligns with the center line of the cone
   * @param semi in the interval [0, pi/2] */
  public Cone2Distance(Tensor xya, Scalar semi) {
    inverse = new Se2Bijection(xya).inverse();
    GlobalAssert.that(Scalars.lessEquals(semi, Cone2Region.PI_HALF));
    this.semi = Sign.requirePositiveOrZero(semi);
    semi_pi_half = semi.add(Cone2Region.PI_HALF);
    normal = AngleVector.of(semi_pi_half);
  }

  @Override // from TensorScalarFunction
  public Scalar apply(Tensor tensor) {
    Tensor local = inverse.apply(tensor);
    local.set(Scalar::abs, 1);
    Scalar angle = ArcTan.of(local.Get(0), local.Get(1));
    if (Scalars.lessThan(angle, semi))
      return RealScalar.ZERO;
    if (Scalars.lessThan(semi_pi_half, angle))
      return Norm._2.ofVector(local);
    return local.dot(normal).Get();
  }
}
