// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.demo.car.box.CHatchbackModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

/** DO NOT CHANGE THE VALUES IN THE EXISTING FUNCTIONS */
public enum CarStatic {
  ;
  private static final double Dz1 = 0.05; // TODO not final code design

  public static final Scalar noSlipRate(Scalar speed, Scalar radius) {
    return speed.divide(radius);
  }

  /** {8.383333333333333, 0,
   * 0, 1,
   * -50, -75,
   * 25.794871794871792, 25.794871794871792, 25.794871794871792, 25.794871794871792}
   * 
   * @return */
  public static CarState x0_demo1() {
    VehicleModel params = CHatchbackModel.standard();
    Scalar speed = RealScalar.of(30 + 3.6 * Dz1);
    return new CarState(Tensors.vector( //
        speed.number().doubleValue() / 3.6, // Ux
        0, // Uy
        0, // r
        1, // Ksi
        // ---
        -50, -75, // px, py
        // ---
        noSlipRate(speed, params.tire(0).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, params.tire(1).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, params.tire(2).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, params.tire(3).radius()).number().doubleValue() / 3.6 //
    ));
  }

  /** {8.383333333333333, 2,
   * 0, 1,
   * -50, -75,
   * 25.794871794871792, 25.794871794871792, 25.794871794871792, 25.794871794871792}
   * 
   * @return */
  public static CarState x0_demo2() {
    VehicleModel params = CHatchbackModel.standard();
    Scalar speed = RealScalar.of(30 + 3.6 * Dz1);
    return new CarState(Tensors.vector( //
        speed.number().doubleValue() / 3.6, // Ux
        2, // Uy
        0, // r
        1, // Ksi
        // ---
        -50, -75, // px, py
        // ---
        noSlipRate(speed, params.tire(0).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, params.tire(1).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, params.tire(2).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, params.tire(3).radius()).number().doubleValue() / 3.6 //
    ));
  }

  /** @return */
  public static CarState x0_demo3() {
    VehicleModel params = CHatchbackModel.standard();
    Scalar speed = RealScalar.of(20 + 3.6 * Dz1);
    return new CarState(Tensors.vector( //
        speed.number().doubleValue() / 3.6, // Ux
        .3, // Uy
        .4, // r
        -.5, // Ksi
        // ---
        -50, -75, // px, py
        // ---
        noSlipRate(speed, params.tire(0).radius()).number().doubleValue() / 3.6 + 3, //
        noSlipRate(speed, params.tire(1).radius()).number().doubleValue() / 3.6 - 2, //
        noSlipRate(speed, params.tire(2).radius()).number().doubleValue() / 3.6 + 4, //
        noSlipRate(speed, params.tire(3).radius()).number().doubleValue() / 3.6 - 5 //
    ));
  }
}
