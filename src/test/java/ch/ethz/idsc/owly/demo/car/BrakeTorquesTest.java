// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class BrakeTorquesTest extends TestCase {
  public void testSimple() {
    CarModel carModel = CHatchbackModel.standard();
    CarState carState = CarStatic.x0_demo1();
    CarControl cc = carModel.createControl(Tensors.vector(0, 1, 0, 0));
    Scalar mu = RealScalar.of(0.8); // friction coefficient on dry road
    TireForces tireForces = new TireForces(carModel, carState, cc, mu);
    BrakeTorques brakes = new BrakeTorques(carModel, carState, cc, tireForces);
    // System.out.println(brakes.asVector());
    assertEquals(brakes.Tb1L, brakes.Tb1R);
    assertTrue(Scalars.lessThan(brakes.Tb1L, RealScalar.ZERO));
    assertEquals(brakes.Tb2L, brakes.Tb2R);
    assertTrue(Scalars.lessThan(brakes.Tb2L, RealScalar.ZERO));
  }
}
