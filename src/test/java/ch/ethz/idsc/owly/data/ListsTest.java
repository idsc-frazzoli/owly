// code by jph
package ch.ethz.idsc.owly.data;

import java.util.LinkedList;

import junit.framework.TestCase;

public class ListsTest extends TestCase {
  public void testFail() {
    try {
      Lists.getLast(new LinkedList<>());
      assertTrue(false);
    } catch (Exception exception) {
      // ---
    }
  }
}
