// code by jph
package ch.ethz.idsc.owly.demo.rice;

import ch.ethz.idsc.tensor.RealScalar;
import junit.framework.TestCase;

public class Rice1StateSpaceModelTest extends TestCase {
  public void testFails() {
    try {
      new Rice1StateSpaceModel(RealScalar.ZERO);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
    try {
      new Rice1StateSpaceModel(RealScalar.of(-1));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
