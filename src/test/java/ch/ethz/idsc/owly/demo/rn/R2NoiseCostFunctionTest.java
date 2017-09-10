// code by jph
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.tensor.RealScalar;
import junit.framework.TestCase;

public class R2NoiseCostFunctionTest extends TestCase {
  public void testSimple() {
    CostFunction costFunction = new R2NoiseCostFunction(RealScalar.of(.2));
    assertFalse(HeuristicQ.of(costFunction));
  }
}
