// code by jph
package ch.ethz.idsc.owly.demo.drift;

import java.util.Collection;

import ch.ethz.idsc.owly.math.flow.Flow;
import junit.framework.TestCase;

public class DriftControlsTest extends TestCase {
  public void testSimple() {
    Collection<Flow> controls = DriftControls.create(10);
    assertEquals(controls.size(), 11);
  }

  public void testMakeOdd() {
    Collection<Flow> controls = DriftControls.create(11);
    assertEquals(controls.size(), 13);
  }
}
