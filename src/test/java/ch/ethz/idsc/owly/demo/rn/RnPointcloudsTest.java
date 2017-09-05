// code by jph
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RnPointcloudsTest extends TestCase {
  public void testSimple2D() {
    Region region = RnPointclouds.createRandomRegion(1, Tensors.vector(10, 10), Tensors.vector(1, 1), RealScalar.of(1.5));
    assertTrue(region.isMember(Tensors.vector(10.5, 10.5)));
    assertTrue(region.isMember(Tensors.vector(10, 10)));
    assertFalse(region.isMember(Tensors.vector(8, 8)));
    assertFalse(region.isMember(Tensors.vector(13, 13)));
  }
}
