// code by jph
package ch.ethz.idsc.owly.demo.psu.glc;

import ch.ethz.idsc.owly.demo.psu.glc.PsuDemo;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import junit.framework.TestCase;

public class PsuDemoTest extends TestCase {
  public void testFindGoal() {
    TrajectoryPlanner trajectoryPlanner = PsuDemo.simple();
    assertTrue(trajectoryPlanner.getBest().isPresent());
  }
}
