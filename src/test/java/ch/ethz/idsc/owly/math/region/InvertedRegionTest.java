// code by jph
package ch.ethz.idsc.owly.math.region;

import junit.framework.TestCase;

public class InvertedRegionTest extends TestCase {
  public void testSimple() {
    assertTrue(new InvertedRegion(new EmptyRegion()).isMember(null));
  }
}
