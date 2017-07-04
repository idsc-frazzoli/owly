// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.demo.car.box.CHatchbackModel;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class MotorTorquesTest extends TestCase {
  public void testSome() {
    VehicleModel vehicleModel = CHatchbackModel.standard();
    // Scalar throttle = RealScalar.of(200);
    CarControl carControl = vehicleModel.createControl(Tensors.vector(0, 0, 0, .123));
    // Tensor torques = MotorTorques.standard(gammaM, throttle)(params, throttle);
    // // System.out.println(motorTorques.asVector());
    assertEquals(carControl.throttleV.Get(0), carControl.throttleV.Get(1));
    assertEquals(carControl.throttleV.Get(2), carControl.throttleV.Get(3));
    // System.out.println(cc.throttleV);
    // assertEquals(Total.of(motorTorques.asVector()), throttle);
  }
}
