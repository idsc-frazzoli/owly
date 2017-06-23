// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

public class BrakeTorques {
  final Scalar Tb1L; // 1
  final Scalar Tb1R; // 2
  final Scalar Tb2L; // 3
  final Scalar Tb2R; // 4

  public BrakeTorques(CarState cs, CarControl cc, TireForces tireForces) {
    // TODO
    Tb1L = RealScalar.ZERO;
    Tb1R = RealScalar.ZERO;
    Tb2L = RealScalar.ZERO;
    Tb2R = RealScalar.ZERO;
  }
}
