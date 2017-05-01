// code by jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.ZeroScalar;
import junit.framework.TestCase;

public class DomainQueueTest extends TestCase {
  public void testQueue() {
    DomainQueue dq = new DomainQueue(new Node(null, null, RealScalar.of(1), ZeroScalar.get()));
    dq.add(new Node(null, null, RealScalar.of(0), ZeroScalar.get()));
    dq.add(new Node(null, null, RealScalar.of(9), ZeroScalar.get()));
    Node n1 = dq.poll();
    assertEquals(n1.cost().number(), 0);
    Node n2 = dq.poll();
    assertEquals(n2.cost().number(), 1);
    Node n3 = dq.poll();
    assertEquals(n3.cost().number(), 9);
    assertTrue(dq.isEmpty());
  }
}
