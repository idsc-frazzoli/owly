// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class TireForcesTest extends TestCase {
  public void testSimple() {
    CarModel carModel = new CHatchbackModel();
    CarState carState = CarStatic.x0_demo1();
    CarControl carControl = new CarControl(Tensors.vector(0, 0, 0, 0));
    TireForces tireForces = new TireForces(carModel, carState, carControl);
    assertTrue(Chop.isZeros(tireForces.asVectorFX()));
    assertTrue(Chop.isZeros(tireForces.asVectorFY()));
    assertTrue(Chop.isZeros(tireForces.asVector_fX()));
    assertTrue(Chop.isZeros(tireForces.asVector_fY()));
    Tensor Fz = tireForces.asVectorFZ();
    assertEquals(Fz.Get(0), Fz.Get(1));
    assertEquals(Fz.Get(2), Fz.Get(3));
  }
}
