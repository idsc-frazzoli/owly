// code by jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.owly.math.flow.EulerIntegrator;
import ch.ethz.idsc.tensor.RealScalar;
import junit.framework.TestCase;

public class FixedStateIntegratorTest extends TestCase {
  public void testFail1() {
    try {
      FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RealScalar.of(-.1), 3);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }

  public void testFail2() {
    try {
      FixedStateIntegrator.create(EulerIntegrator.INSTANCE, RealScalar.of(0), 3);
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
