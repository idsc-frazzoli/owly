// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import junit.framework.TestCase;

public class MotorTorquesTest extends TestCase {
  public void testSome() {
    CarModel params = new CHatchbackModel();
    MotorTorques motorTorques = new MotorTorques(params, RealScalar.of(.2));
    // System.out.println(motorTorques.asVector());
    assertEquals(motorTorques.Tm1L, motorTorques.Tm1R);
    assertEquals(motorTorques.Tm2L, motorTorques.Tm2R);
    // TODO check sum == max * .2
  }
}
