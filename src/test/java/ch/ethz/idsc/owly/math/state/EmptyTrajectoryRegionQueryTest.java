// code by jph
package ch.ethz.idsc.owly.math.state;

import junit.framework.TestCase;

public class EmptyTrajectoryRegionQueryTest extends TestCase {
  public void testSimple() {
    assertEquals( //
        EmptyTrajectoryRegionQuery.INSTANCE.firstMember(null), //
        TrajectoryRegionQuery.NOMATCH);
  }
}
