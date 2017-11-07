// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class NegativeHalfspaceRegionTest extends TestCase {
  public void testSimple() {
    TensorRegion r = new NegativeHalfspaceRegion(1);
    assertFalse(r.isMember(Tensors.vector(1, +1, 1)));
    assertTrue(r.isMember(Tensors.vector(1, -1, 1)));
  }
}
