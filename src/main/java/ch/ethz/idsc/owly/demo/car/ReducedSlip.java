// code by edo and jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.Normalize2D;
import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Sin;

/** robust computation of slip */
public class ReducedSlip implements SlipInterface {
  private static final Scalar PI_HALF = RealScalar.of(Math.PI / 2);
  // ---
  private final Tensor mu;

  /** if U == {rtw, 0} that means no slip
   * 
   * @param pacejka3
   * @param U ground speed in coordinate system of tire
   * @param rtw == radius * rate of wheel */
  public ReducedSlip(Pacejka3 pacejka3, Scalar factor, Tensor U, Scalar rtw) {
    final Scalar ux = U.Get(0).subtract(rtw); // effective speed of tire (longitude)
    final Scalar uy = U.Get(1);
    final Scalar value;
    if (Scalars.isZero(rtw))
      /** the limit case has been established with Mathematica.
       * for reasonable pacejka constants, the "total" variable will evaluate to be positive.
       * due to the negate the final friction "mu" is then pointing against the ground speed {ux, uy} */
      value = PI_HALF;
    else
      value = ArcTan.of(pacejka3.B.multiply(Hypot.bifunction.apply(ux, uy).divide(rtw)));
    Scalar total = pacejka3.D.multiply(Sin.of(pacejka3.C.multiply(value))).multiply(factor);
    mu = Normalize2D.unlessZero(ux, uy).multiply(total.negate());
  }

  @Override
  public Tensor slip() {
    return mu;
  }
}
