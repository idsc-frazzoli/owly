// code by edo and jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Normalize;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.red.Norm;

/** robust computation of slip */
public class RobustSlip implements SlipInterface {
  private final Tensor mu;

  /** if U == {rtw, 0} that means no slip
   * 
   * @param pacejka3
   * @param U ground speed in coordinate system of tire
   * @param rtw == radius * rate of wheel */
  public RobustSlip(Pacejka3 pacejka3, Tensor U, Scalar rtw) {
    final Scalar ux = U.Get(0).subtract(rtw); // effective speed of tire (longitude)
    final Scalar uy = U.Get(1);
    final Scalar total = Scalars.isZero(rtw) ? //
        pacejka3.limit() : pacejka3.apply(Hypot.of(ux, uy).divide(rtw));
    mu = Normalize.unlessZero(Tensors.of(ux, uy), Norm._2).multiply(total.negate());
  }

  @Override
  public Tensor slip() {
    return mu;
  }
}
