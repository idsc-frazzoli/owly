// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.demo.car.box.CHatchbackModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;

/** DO NOT CHANGE THE VALUES IN THE EXISTING FUNCTIONS */
public enum CarStatic {
  ;
  /** constant motivated by previous design. not relevant for application layer */
  private static final double Dz1 = 0.05;

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
    VehicleModel vehicleModel = CHatchbackModel.standard();
    Scalar speed = RealScalar.of(30 + 3.6 * Dz1);
    return new CarState(Tensors.vector( //
        speed.number().doubleValue() / 3.6, // Ux
        0, // Uy
        0, // r
        1, // Ksi
        // ---
        -50, -75, // px, py
        // ---
        noSlipRate(speed, vehicleModel.wheel(0).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, vehicleModel.wheel(1).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, vehicleModel.wheel(2).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, vehicleModel.wheel(3).radius()).number().doubleValue() / 3.6 //
    ));
  }

  /** {8.383333333333333, 2,
   * 0, 1,
   * -50, -75,
   * 25.794871794871792, 25.794871794871792, 25.794871794871792, 25.794871794871792}
   * 
   * @return */
  public static CarState x0_demo2() {
    VehicleModel vehicleModel = CHatchbackModel.standard();
    Scalar speed = RealScalar.of(30 + 3.6 * Dz1);
    return new CarState(Tensors.vector( //
        speed.number().doubleValue() / 3.6, // Ux
        2, // Uy
        0, // r
        1, // Ksi
        // ---
        -50, -75, // px, py
        // ---
        noSlipRate(speed, vehicleModel.wheel(0).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, vehicleModel.wheel(1).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, vehicleModel.wheel(2).radius()).number().doubleValue() / 3.6, //
        noSlipRate(speed, vehicleModel.wheel(3).radius()).number().doubleValue() / 3.6 //
    ));
  }

  /** @return */
  public static CarState x0_demo3() {
    VehicleModel vehicleModel = CHatchbackModel.standard();
    Scalar speed = RealScalar.of(20 + 3.6 * Dz1);
    return new CarState(Tensors.vector( //
        speed.number().doubleValue() / 3.6, // Ux
        .3, // Uy
        .4, // r
        -.5, // Ksi
        // ---
        -50, -75, // px, py
        // ---
        noSlipRate(speed, vehicleModel.wheel(0).radius()).number().doubleValue() / 3.6 + 3, //
        noSlipRate(speed, vehicleModel.wheel(1).radius()).number().doubleValue() / 3.6 - 2, //
        noSlipRate(speed, vehicleModel.wheel(2).radius()).number().doubleValue() / 3.6 + 4, //
        noSlipRate(speed, vehicleModel.wheel(3).radius()).number().doubleValue() / 3.6 - 5 //
    ));
  }
}
