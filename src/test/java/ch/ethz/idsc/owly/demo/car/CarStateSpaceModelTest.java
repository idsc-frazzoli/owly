// code by jph
package ch.ethz.idsc.owly.demo.car;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class CarStateSpaceModelTest extends TestCase {
  public void testSimple() {
    CarModel carModel = CHatchbackModel.standard();
    CarState carState = CarStatic.x0_demo3();
    CarControl carControl = carModel.createControl(Tensors.vector(0.4, .03, 0, .4));
    StateSpaceModel ssm = new CarStateSpaceModel(carModel);
    Tensor res = ssm.f(carState.asVector(), carControl.asVector());
    Tensor gnd = Tensors.fromString(
        "{1.4243749104432206, 2.1618367009520716, -1.8297547914814403, 0.4, 5.0631654668445165, -2.4241717228308928, -1231.189048069456, 764.091380010576, -981.052814181167, 268.57867140004987}");
    // assertTrue(Chop.isZeros(res.subtract(gnd)));
  }
}
