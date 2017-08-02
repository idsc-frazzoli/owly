// code by jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class StateTimeTest extends TestCase {
  public void testSimple() {
    StateTime s1 = new StateTime(Tensors.vector(1, 0, 1), RealScalar.of(2));
    StateTime s2 = new StateTime(Tensors.vector(1, 0, 1), RealScalar.of(2));
    assertEquals(s1, s2);
  }
}
