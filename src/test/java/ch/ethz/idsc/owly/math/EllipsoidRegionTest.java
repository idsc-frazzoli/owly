// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class EllipsoidRegionTest extends TestCase {
  public void testSimple() {
    Region region = new EllipsoidRegion(Tensors.vector(10, 5), Tensors.vector(1, 1));
    assertTrue(region.isMember(Tensors.vector(10, 5)));
    assertTrue(region.isMember(Tensors.vector(10, 5.5)));
    assertTrue(region.isMember(Tensors.vector(10, 6)));
    assertFalse(region.isMember(Tensors.vector(10, 6.5)));
  }

  public void testSimple2() {
    Region region = new EllipsoidRegion(Tensors.vector(10, 5), Tensors.vector(2, 2));
    assertTrue(region.isMember(Tensors.vector(10, 5)));
    assertTrue(region.isMember(Tensors.vector(10, 5.5)));
    assertTrue(region.isMember(Tensors.vector(10, 7)));
    assertTrue(region.isMember(Tensors.vector(12, 5)));
    assertTrue(region.isMember(Tensors.vector(11.2, 6.2)));
    assertFalse(region.isMember(Tensors.vector(10, 7.1)));
    assertFalse(region.isMember(Tensors.vector(10, 7.5)));
  }

  public void testEllipsoid() {
    Region region = new EllipsoidRegion(Tensors.vector(10, 5), Tensors.vector(2, 1));
    assertTrue(region.isMember(Tensors.vector(10, 5)));
    assertTrue(region.isMember(Tensors.vector(10, 5.5)));
    assertFalse(region.isMember(Tensors.vector(10, 7)));
    assertTrue(region.isMember(Tensors.vector(12, 5)));
    assertFalse(region.isMember(Tensors.vector(12.1, 5)));
    assertFalse(region.isMember(Tensors.vector(11.2, 6.2)));
    assertFalse(region.isMember(Tensors.vector(10, 6.1)));
    assertFalse(region.isMember(Tensors.vector(10, 7.5)));
  }
}
