// code by jph
package ch.ethz.idsc.owly.demo.util;

import junit.framework.TestCase;

public class UserHomeTest extends TestCase {
  public void testSimple() {
    assertTrue(UserHome.file("").isDirectory());
  }
}
