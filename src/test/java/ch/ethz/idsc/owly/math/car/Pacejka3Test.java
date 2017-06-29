// code by jph
package ch.ethz.idsc.owly.math.car;

import ch.ethz.idsc.owly.demo.car.CHatchbackModel;
import ch.ethz.idsc.owly.demo.car.CarModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import junit.framework.TestCase;

public class Pacejka3Test extends TestCase {
  public void testSimple() {
    CarModel c = new CHatchbackModel();
    Scalar r = c.pacejka(0).apply(RealScalar.ZERO);
    assertEquals(r, RealScalar.ZERO);
  }

  public void testAntiSymmetric() {
    CarModel c = new CHatchbackModel();
    Scalar sp = c.pacejka(1).apply(RealScalar.ONE);
    Scalar sn = c.pacejka(0).apply(RealScalar.ONE.negate());
    assertEquals(sp, sn.negate());
    assertTrue(sp.toString().startsWith("0.854"));
  }
}
