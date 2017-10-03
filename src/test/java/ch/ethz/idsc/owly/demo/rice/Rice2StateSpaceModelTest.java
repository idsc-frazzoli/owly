// code by jph
package ch.ethz.idsc.owly.demo.rice;

import ch.ethz.idsc.tensor.RealScalar;
import junit.framework.TestCase;

public class Rice2StateSpaceModelTest extends TestCase {
  public void testFails() {
    try {
      new Rice2StateSpaceModel(RealScalar.ZERO);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
    try {
      new Rice2StateSpaceModel(RealScalar.of(-1));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
