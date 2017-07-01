// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.red.Total;
import junit.framework.TestCase;

public class MotorTorquesTest extends TestCase {
  public void testSome() {
    VehicleModel params = CHatchbackModel.standard();
    Scalar throttle = RealScalar.of(200);
    MotorTorques motorTorques = new MotorTorques(params, throttle);
    // System.out.println(motorTorques.asVector());
    assertEquals(motorTorques.Tm1L, motorTorques.Tm1R);
    assertEquals(motorTorques.Tm2L, motorTorques.Tm2R);
    assertEquals(Total.of(motorTorques.asVector()), throttle);
  }
}
