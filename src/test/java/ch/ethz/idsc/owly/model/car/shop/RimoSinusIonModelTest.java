// code by jph
package ch.ethz.idsc.owly.model.car.shop;

import ch.ethz.idsc.owly.model.car.VehicleModel;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Dimensions;
import junit.framework.TestCase;

public class RimoSinusIonModelTest extends TestCase {
  public void testSimple() {
    VehicleModel vm = RimoSinusIonModel.standard();
    Tensor fp = vm.footprint();
    assertEquals(Dimensions.of(fp).get(1), (Integer) 3);
  }
}
