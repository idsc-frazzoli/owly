// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

public enum CarSteering {
  FRONT(RealScalar.ONE), // Ackermann
  FRONT_PARALLEL(RealScalar.ONE), // simple, only recommended for tests
  REAR(RealScalar.ONE), // Ackermann
  BOTH(RealScalar.of(.5)), // Ackermann
  ;
  private CarSteering(Scalar factor) {
    this.factor = factor;
  }

  public final Scalar factor;
}
