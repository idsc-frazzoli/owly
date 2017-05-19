// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.ZeroScalar;
import junit.framework.TestCase;

public class DomainQueueTest extends TestCase {
  public void testQueue() {
    DomainQueue dq = new DomainQueue();
    dq.add(new GlcNode(null, null, RealScalar.of(1), ZeroScalar.get()));
    dq.add(new GlcNode(null, null, RealScalar.of(0), ZeroScalar.get()));
    dq.add(new GlcNode(null, null, RealScalar.of(9), ZeroScalar.get()));
    GlcNode n1 = dq.poll();
    assertEquals(n1.costFromRoot().number(), 0);
    GlcNode n2 = dq.poll();
    assertEquals(n2.costFromRoot().number(), 1);
    GlcNode n3 = dq.poll();
    assertEquals(n3.costFromRoot().number(), 9);
    assertTrue(dq.isEmpty());
  }

  public void testMap() {
    Map<Integer, String> map = new HashMap<>();
    map.put(1, "one");
    map.put(2, "two");
    map.put(3, "three");
    Collection<String> asd = map.values();
    asd.remove("two");
    System.out.println(map.size());
    System.out.println(map);
  }
}
