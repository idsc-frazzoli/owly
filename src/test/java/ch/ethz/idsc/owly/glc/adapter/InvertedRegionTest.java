// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.InvertedRegion;
import junit.framework.TestCase;

public class InvertedRegionTest extends TestCase {
  public void testSimple() {
    assertTrue(new InvertedRegion(new EmptyRegion()).isMember(null));
  }
}
