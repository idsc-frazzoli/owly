// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

public abstract class DefaultCarModel implements CarModel {
  public final Scalar gForce() {
    return mass().multiply(RealScalar.of(9.81));
  }
}
