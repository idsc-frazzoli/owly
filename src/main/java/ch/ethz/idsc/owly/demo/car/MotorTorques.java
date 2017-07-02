// code by edo
// code adapted by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class MotorTorques {
  private final Scalar HALF = RealScalar.of(.5);
  // ---
  final Scalar Tm1L; // 1
  final Scalar Tm1R; // 2
  final Scalar Tm2L; // 3
  final Scalar Tm2R; // 4

  /** @param params
   * @param throttle absolute [Nm] */
  public MotorTorques(VehicleModel params, Scalar throttle) {
    // ---
    final Scalar reqTorque = throttle.multiply(HALF);
    final Scalar rearCoeff = params.gammaM();
    final Scalar frontCoeff = RealScalar.ONE.subtract(rearCoeff);
    Tm1L = frontCoeff.multiply(reqTorque);
    Tm1R = Tm1L;
    Tm2L = rearCoeff.multiply(reqTorque);
    Tm2R = Tm2L;
  }

  public Tensor asVector() {
    return Tensors.of(Tm1L, Tm1R, Tm2L, Tm2R);
  }
}
