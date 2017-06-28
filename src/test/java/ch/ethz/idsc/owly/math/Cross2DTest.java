// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Cross2DTest extends TestCase {
  public void testSimple() {
    // Cross[{1, 2}] == {-2, 1}
    assertEquals(Cross2D.of(Tensors.vector(1, 2)), Tensors.vector(-2, 1));
  }
}
