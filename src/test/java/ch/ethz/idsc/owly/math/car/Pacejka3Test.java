// code by jph
package ch.ethz.idsc.owly.math.car;

import ch.ethz.idsc.owly.demo.car.VehicleModel;
import ch.ethz.idsc.owly.demo.car.box.CHatchbackModel;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import junit.framework.TestCase;

public class Pacejka3Test extends TestCase {
  public void testSimple() {
    VehicleModel c = CHatchbackModel.standard();
    Scalar r = c.tire(0).pacejka().apply(RealScalar.ZERO);
    assertEquals(r, RealScalar.ZERO);
  }

  public void testAntiSymmetric() {
    VehicleModel c = CHatchbackModel.standard();
    Scalar sp = c.tire(1).pacejka().apply(RealScalar.ONE);
    Scalar sn = c.tire(0).pacejka().apply(RealScalar.ONE.negate());
    assertEquals(sp, sn.negate());
    assertTrue(sp.toString().startsWith("0.854"));
  }

  public void testFail() {
    VehicleModel carModel = CHatchbackModel.standard();
    try {
      carModel.tire(0).pacejka().apply(DoubleScalar.POSITIVE_INFINITY);
    } catch (Exception exception) {
      // ---
    }
  }
}
