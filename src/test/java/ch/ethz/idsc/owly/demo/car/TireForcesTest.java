// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class TireForcesTest extends TestCase {
  public void testDemo1() {
//    System.out.println("TireForcesTest::demo1");
    CarModel carModel = new CHatchbackModel();
    CarState carState = CarStatic.x0_demo1();
    // System.out.println(carState.asVector());
    CarControl carControl = carModel.createControl(Tensors.vector(0, 0, 0, 0));
    TireForces tireForces = new TireForces(carModel, carState, carControl);
    assertTrue(Chop.isZeros(tireForces.asVectorFX()));
    assertTrue(Chop.isZeros(tireForces.asVectorFY()));
    assertTrue(Chop.isZeros(tireForces.asVector_fX()));
    assertTrue(Chop.isZeros(tireForces.asVector_fY()));
    Tensor Fz = tireForces.asVectorFZ();
    assertEquals(Fz.Get(0), Fz.Get(1));
    assertEquals(Fz.Get(2), Fz.Get(3));
  }

  public void testDemo2() {
//    System.out.println("TireForcesTest::demo2");
    CarModel carModel = new CHatchbackModel();
    CarState carState = CarStatic.x0_demo2();
    // System.out.println(carState.asVector());
    CarControl carControl = carModel.createControl(Tensors.vector(0, 0, 0, 0));
    TireForces tireForces = new TireForces(carModel, carState, carControl);
    assertTrue(Chop.isZeros(tireForces.asVectorFX()));
    assertTrue(Chop.isZeros(tireForces.asVector_fX()));
    // ---
    assertTrue(tireForces.asVectorFY().Get(0).toString().startsWith("-2178."));
    assertTrue(tireForces.asVectorFY().Get(1).toString().startsWith("-5086."));
    assertTrue(tireForces.asVectorFY().Get(2).toString().startsWith("-497."));
    assertTrue(tireForces.asVectorFY().Get(3).toString().startsWith("-3439."));
    // ---
    assertTrue(tireForces.asVectorFZ().Get(0).toString().startsWith("2704."));
    assertTrue(tireForces.asVectorFZ().Get(1).toString().startsWith("6315."));
    assertTrue(tireForces.asVectorFZ().Get(2).toString().startsWith("610."));
    assertTrue(tireForces.asVectorFZ().Get(3).toString().startsWith("4221."));
    // ---
    assertTrue(tireForces.asVector_fY().Get(0).toString().startsWith("-2178."));
    assertTrue(tireForces.asVector_fY().Get(1).toString().startsWith("-5086."));
    assertTrue(tireForces.asVector_fY().Get(2).toString().startsWith("-497."));
    assertTrue(tireForces.asVector_fY().Get(3).toString().startsWith("-3439."));
  }

  /** {903.610973774307, -1807.7084631684347, 2736.1718856177345, -865.0967379905104}
   * {4386.855391683335, 1896.4937941266203, 766.4628278978114, 202.1501561222724}
   * {5768.831655182105, 3430.8856422219737, 3494.974357778026, 1157.0283448178939}
   * ---
   * {2896.1637422536455, -677.1858655495801, 2736.1718856177345, -865.0967379905104}
   * {3416.6136154856404, 2530.991486052735, 766.4628278978114, 202.1501561222724}
   * 
   * FORCES = 1.0e+03 *
   * 0.9036
   * -1.8077
   * 2.7362
   * -0.8651
   * 
   * 4.3869
   * 1.8965
   * 0.7665
   * 0.2022
   * 
   * 5.7688
   * 3.4309
   * 3.4950
   * 1.1570
   * 
   * forces = 1.0e+03 *
   * 2.8962
   * -0.6772
   * 2.7362
   * -0.8651
   * 
   * 3.4166
   * 2.5310
   * 0.7665
   * 0.2022 */
  public void testDemo3() {
//    System.out.println("TireForcesTest::demo3");
    CarModel carModel = new CHatchbackModel();
    CarState carState = CarStatic.x0_demo3();
    // System.out.println(carState.asVector());
    double maxDelta = 30 * Math.PI / 180;
    CarControl carControl = carModel.createControl(Tensors.vector(.5 / maxDelta, 0, 0, 0));
    // new CarControl();
    @SuppressWarnings("unused")
    TireForces tireForces = new TireForces(carModel, carState, carControl);
    // CONFIRMED
    // System.out.println(tireForces.asVectorFX());
    // System.out.println(tireForces.asVectorFY());
    // System.out.println(tireForces.asVectorFZ());
    // // ---
    // System.out.println("---");
    // System.out.println(tireForces.asVector_fX());
    // System.out.println(tireForces.asVector_fY());
  }
}
