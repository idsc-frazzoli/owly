// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.region.EmptyRegion;
import junit.framework.TestCase;

public class EmptyRegionTest extends TestCase {
  public void testSimple() {
    assertFalse(new EmptyRegion().isMember(null));
  }
}
