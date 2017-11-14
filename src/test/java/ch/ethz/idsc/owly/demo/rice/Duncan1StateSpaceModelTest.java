// code by jph
package ch.ethz.idsc.owly.demo.rice;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Quantity;
import junit.framework.TestCase;

public class Duncan1StateSpaceModelTest extends TestCase {
  public void testScalar() {
    StateSpaceModel stateSpaceModel = new Duncan1StateSpaceModel(Quantity.of(0.1, "s^-1"));
    Tensor x = Tensors.fromString("{10[m*s^-1], 20[m*s^-1]}");
    Tensor u = Tensors.fromString("{-1[m*s^-2], -1[m*s^-2]}");
    Tensor fxu = stateSpaceModel.f(x, u).multiply(Quantity.of(1, "s"));
    assertEquals(fxu, Tensors.fromString("{-2[m*s^-1], -3[m*s^-1]}"));
  }

  public void testZero() {
    StateSpaceModel stateSpaceModel = new Duncan1StateSpaceModel(Quantity.of(0.0, "s^-1"));
    Tensor x = Tensors.fromString("{10[m*s^-1], 20[m*s^-1]}");
    Tensor u = Tensors.fromString("{-1[m*s^-2], -1[m*s^-2]}");
    Tensor fxu = stateSpaceModel.f(x, u).multiply(Quantity.of(1, "s"));
    assertEquals(fxu, Tensors.fromString("{-1[m*s^-1], -1[m*s^-1]}"));
  }

  public void testFail() {
    try {
      new Duncan1StateSpaceModel(Quantity.of(-1.0, "s^-1"));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
