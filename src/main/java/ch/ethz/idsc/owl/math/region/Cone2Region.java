// code by jph
package ch.ethz.idsc.owl.math.region;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.math.map.Se2Bijection;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Clip;

/** planar infinite cone region */
public class Cone2Region implements Region<Tensor> {
  private static final Scalar BOUND = RealScalar.of(Math.PI / 2);
  // ---
  private final TensorUnaryOperator inverse;
  private final Clip clip;

  /** @param xya vector of the form {x,y,angle} where {x,y} is the tip of the cone
   * and angle aligns with the center line of the cone
   * @param semi in the interval [0, pi/2] */
  /* package */ Cone2Region(Tensor xya, Scalar semi) {
    inverse = new Se2Bijection(xya).inverse();
    GlobalAssert.that(Scalars.lessEquals(semi, BOUND));
    clip = Clip.function(semi.negate(), semi);
  }

  @Override
  public boolean isMember(Tensor tensor) {
    Tensor local = inverse.apply(tensor);
    return clip.isInside(ArcTan.of(local.Get(0), local.Get(1)));
  }
}
