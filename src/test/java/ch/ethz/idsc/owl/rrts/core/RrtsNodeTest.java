// code by jph
package ch.ethz.idsc.owl.rrts.core;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RrtsNodeTest extends TestCase {
  public void testSome() {
    RrtsNode root = RrtsNode.createRoot(Tensors.vector(0), RealScalar.ZERO);
    RrtsNode n1 = root.connectTo(Tensors.vector(1), RealScalar.of(10));
    RrtsNode n2 = n1.connectTo(Tensors.vector(2), RealScalar.of(10 + 1));
    RrtsNode n4 = n2.connectTo(Tensors.vector(4), RealScalar.of(10 + 1 + 2));
    RrtsNode n3 = n1.connectTo(Tensors.vector(3), RealScalar.of(10 + 4));
    RrtsNode nm = root.connectTo(Tensors.vector(.5), RealScalar.of(.5));
    assertEquals(n1.costFromRoot(), RealScalar.of(10));
    assertEquals(n2.costFromRoot(), RealScalar.of(11));
    assertEquals(n4.costFromRoot(), RealScalar.of(13));
    assertEquals(n3.costFromRoot(), RealScalar.of(14));
    assertEquals(nm.costFromRoot(), RealScalar.of(.5));
    nm.rewireTo(n1, RealScalar.of(2));
    assertEquals(n1.costFromRoot(), RealScalar.of(2.5));
    assertEquals(n2.costFromRoot(), RealScalar.of(3.5));
    assertEquals(n4.costFromRoot(), RealScalar.of(5.5));
    assertEquals(n3.costFromRoot(), RealScalar.of(6.5));
    assertEquals(nm.costFromRoot(), RealScalar.of(.5));
  }
}
