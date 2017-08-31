// code by jph
package ch.ethz.idsc.owly.data;

import junit.framework.TestCase;

public class StopwatchTest extends TestCase {
  public void testSimple() {
    Stopwatch stopwatch = Stopwatch.stopped();
    assertEquals(stopwatch.display_nanoSeconds(), 0);
    assertEquals(stopwatch.display_seconds(), 0.0);
    try {
      stopwatch.stop();
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
    stopwatch.start();
    Math.sin(1);
    assertTrue(0 < stopwatch.display_nanoSeconds());
    stopwatch.stop();
    assertTrue(0 < stopwatch.display_seconds());
    assertEquals(stopwatch.display_seconds(), stopwatch.display_nanoSeconds() * 1e-9);
  }

  public void testStarted() {
    Stopwatch stopwatch = Stopwatch.started();
    try {
      stopwatch.start();
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
    assertTrue(0 < stopwatch.display_nanoSeconds());
  }
}