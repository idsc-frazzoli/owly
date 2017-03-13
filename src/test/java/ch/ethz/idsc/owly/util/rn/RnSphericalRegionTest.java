package ch.ethz.idsc.owly.util.rn;

import ch.ethz.idsc.owly.util.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RnSphericalRegionTest extends TestCase {
  public void testSimple() {
    Region region = new RnSphericalRegion(Tensors.vectorLong(10, 5), RealScalar.of(1));
    assertTrue(region.isMember(Tensors.vectorLong(10, 5)));
    assertTrue(region.isMember(Tensors.vectorDouble(10, 5.5)));
    assertTrue(region.isMember(Tensors.vectorLong(10, 6)));
    assertFalse(region.isMember(Tensors.vectorDouble(10, 6.5)));
  }
}
