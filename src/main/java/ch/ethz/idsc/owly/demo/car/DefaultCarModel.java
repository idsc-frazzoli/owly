// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

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
  public Scalar noSlipRate(Scalar speed) {
    return speed.divide(radius());
  }
}
