// code by jph
package ch.ethz.idsc.owly.demo.se2.twd;

import java.util.Collection;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.qty.Quantity;
import junit.framework.TestCase;

public class TwdForwardFlowsTest extends TestCase {
  public void testSimple() {
    TwdFlows twdFlows = new TwdForwardFlows(Quantity.of(3, "m*s^-1"), Quantity.of(1, "m"));
    int n = 3;
    Collection<Flow> collection = twdFlows.getFlows(n);
    assertEquals(collection.size(), 2 * n + 1);
  }
}
