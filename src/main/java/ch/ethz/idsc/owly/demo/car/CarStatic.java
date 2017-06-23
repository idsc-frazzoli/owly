// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

public enum CarStatic {
  ;
  public static CarState x0_demo1() {
    CarModel params = new CHatchbackModel();
    Scalar speed = RealScalar.of(30 + 3.6 * CarModel.Dz1);
    return new CarState(Tensors.vector( //
        speed.number().doubleValue() / 3.6, // Ux
        0, // Uy
        0, // r
        1, // Ksi
        // ---
        -50, -75, // px, py
        // ---
        params.noSlipRate(speed).number().doubleValue() / 3.6, //
        params.noSlipRate(speed).number().doubleValue() / 3.6, //
        params.noSlipRate(speed).number().doubleValue() / 3.6, //
        params.noSlipRate(speed).number().doubleValue() / 3.6 //
    ));
  }
}
