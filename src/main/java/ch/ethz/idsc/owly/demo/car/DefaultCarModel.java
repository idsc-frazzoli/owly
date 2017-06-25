// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.Sign;

public abstract class DefaultCarModel implements CarModel {
  @Override
  public final Scalar gForce() {
    return mass().multiply(RealScalar.of(9.81));
  }

  @Override
  public final Scalar radiusTimes(Scalar omega) {
    return radius().multiply(omega);
  }

  @Override
  public final Scalar noSlipRate(Scalar speed) {
    return speed.divide(radius());
  }

  @Override
  public final Scalar coulombFriction(Scalar speed) {
    return Sign.of(speed).multiply(b().multiply(speed.abs()).add(fric()));
  }

  /***************************************************/
  /** @return dynamic friction coefficient N/(m/s) */
  public abstract Scalar b();

  /** @return coulomb friction */
  public abstract Scalar fric();
}
