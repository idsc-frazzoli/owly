// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

public class MotorTorques {
  final Scalar Tm1L; // 1
  final Scalar Tm1R; // 2
  final Scalar Tm2L; // 3
  final Scalar Tm2R; // 4

  public MotorTorques(CarControl cc) {
    Tm1L = RealScalar.ZERO;
    Tm1R = RealScalar.ZERO;
    Tm2L = RealScalar.ZERO;
    Tm2R = RealScalar.ZERO;
  }
}
