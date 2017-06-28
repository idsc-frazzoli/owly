// code by edo and jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.sca.ArcTan;
import ch.ethz.idsc.tensor.sca.Sin;

/** robust computation of slip */
@Deprecated
class RobustSlip implements SlipInterface {
  private static final Scalar PI_HALF = RealScalar.of(Math.PI / 2);
  // ---
  private final Scalar mux;
  private final Scalar muy;

  /** if U == (rtw, 0) that means no slip
   * 
   * @param pacejka3
   * @param U ground speed in coordinate system of tire
   * @param rtw == radius * rate of wheel */
  public RobustSlip(Pacejka3 pacejka3, Scalar factor, Tensor U, Scalar rtw) {
    final Scalar vx = U.Get(0);
    final Scalar vy = U.Get(1);
    if (Scalars.isZero(rtw)) {
      mux = pacejka3.D.multiply(Sin.of(pacejka3.C.multiply(PI_HALF))) //
          .multiply(StablePytagoras.of(vx, vy)).negate();
      muy = pacejka3.D.multiply(Sin.of(pacejka3.C.multiply(PI_HALF))) //
          .multiply(StablePytagoras.of(vy, vx)).negate();
    } else {
      final Scalar rtn = rtw.subtract(vx);
      final Scalar hy = Hypot.bifunction.apply(rtn, vy);
      mux = pacejka3.D.multiply(Sin.of(pacejka3.C.multiply(ArcTan.of(pacejka3.B.multiply( //
          hy.divide(rtw)))))).multiply(StablePytagoras.of(rtn, vy)).multiply(factor);
      muy = pacejka3.D.multiply(Sin.of(pacejka3.C.multiply(ArcTan.of(pacejka3.B.multiply( //
          hy.divide(rtw)))))).multiply(StablePytagoras.of(vy, rtn)).negate().multiply(factor);
    }
  }

  @Override
  public Tensor slip() {
    return Tensors.of(mux, muy);
  }
}
