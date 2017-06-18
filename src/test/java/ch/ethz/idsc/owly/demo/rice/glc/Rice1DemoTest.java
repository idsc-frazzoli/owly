// code by jph
package ch.ethz.idsc.owly.demo.rice.glc;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import junit.framework.TestCase;

public class Rice1DemoTest extends TestCase {
  public void testFindGoal() {
    TrajectoryPlanner trajectoryPlanner = Rice1Demo.simple();
    assertTrue(trajectoryPlanner.getBest().isPresent());
  }
}
