// code by jph
package ch.ethz.idsc.owly.math.car;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.sca.Chop;
import junit.framework.TestCase;

public class DifferentialSpeedTest extends TestCase {
  public void testSimple() {
    DifferentialSpeed ds = new DifferentialSpeed(RealScalar.of(1.2), RealScalar.of(.5));
    Scalar v = RealScalar.of(4);
    // confirmed with mathematica
    assertTrue(Chop._10.close(ds.get(v, RealScalar.of(+.3)), RealScalar.of(3.4844395839839613)));
    assertEquals(ds.get(v, RealScalar.ZERO), v);
    assertTrue(Chop._10.close(ds.get(v, RealScalar.of(-.3)), RealScalar.of(4.515560416016039)));
  }

  public void testInverted() {
    DifferentialSpeed ds = new DifferentialSpeed(RealScalar.of(1.2), RealScalar.of(-.5));
    Scalar v = RealScalar.of(4);
    // confirmed with mathematica
    assertTrue(Chop._10.close(ds.get(v, RealScalar.of(+.3)), RealScalar.of(4.515560416016039)));
    assertEquals(ds.get(v, RealScalar.ZERO), v);
    assertTrue(Chop._10.close(ds.get(v, RealScalar.of(-.3)), RealScalar.of(3.4844395839839613)));
  }

  public void testFail() {
    try {
      new DifferentialSpeed(RealScalar.of(0.0), RealScalar.of(.5));
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
