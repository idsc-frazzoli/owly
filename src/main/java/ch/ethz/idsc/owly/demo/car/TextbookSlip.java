// code by edo and jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.car.Pacejka3;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Hypot;

/** slip as introduced in textbook
 * 
 * implementation suffers from numerical badness */
class TextbookSlip implements SlipInterface {
  private static final Scalar eps = RealScalar.of(1e-8);
  // ---
  private final Scalar mux;
  private final Scalar muy;

  // TODO API not finalized
  /** if U == (rtw, 0) that means no slip TODO confirm
   * 
   * @param pacejka3
   * @param U ground speed in coordinate system of tire
   * @param rtw == radius * rate of wheel */
  public TextbookSlip(Pacejka3 pacejka3, Tensor U, Scalar rtw) {
    final Scalar vx = U.Get(0);
    final Scalar vy = U.Get(1);
    final Scalar sx = vx.subtract(rtw).divide(rtw); // FIXME div by 0
    final Scalar sy = RealScalar.ONE.add(sx).multiply(vy.divide(vx));
    final Scalar s = Hypot.bifunction.apply(sx, sy);
    final Scalar mu = pacejka3.apply(s);
    mux = mu.multiply(robustDiv(sx, s, eps)).negate();
    muy = mu.multiply(robustDiv(sy, s, eps)).negate();
  }

  @Override
  public Tensor slip() {
    return Tensors.of(mux, muy);
  }

  private static Scalar robustDiv(Scalar num, Scalar den, Scalar eps) {
    if (Scalars.isZero(den)) {
      System.out.println("ROBUST DIV " + num);
      if (Scalars.nonZero(num))
        return num.divide(eps);
      return RealScalar.ZERO;
    }
    return num.divide(den);
  }
}
