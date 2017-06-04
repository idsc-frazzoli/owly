// code by jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class GlcNodeTest extends TestCase {
  public void testCompare() {
    StateTime state1 = new StateTime(Tensors.vector(3, 0), RealScalar.of(3));
    StateTime state2 = new StateTime(Tensors.vector(3, 0), RealScalar.of(3));
    Scalar cost1 = RealScalar.of(1);
    Scalar cost2 = RealScalar.of(1);
    Scalar heuristic1 = RealScalar.of(0);
    Scalar heuristic2 = RealScalar.of(0);
    GlcNode test1 = new GlcNode(null, state1, cost1, heuristic1);
    GlcNode test2 = new GlcNode(null, state1, cost1, heuristic1);
    assertTrue(state1.equals(state1));
    assertTrue(state1.equals(state2));
    // reflexiv
    assertFalse(test1.equals(null));
    // Nodes are completely identical
    assertTrue(test1.equals(test1));
    // Symetrie check
    // assertTrue(test1.equals(test2));
    // assertTrue(test2.equals(test1));
    test2.setMinCostToGoal(heuristic2);
    // Nodes are identically except heuristic
    // assertTrue(test1.equals(test2));
    // Cost is different ==> different node
    GlcNode test3 = new GlcNode(null, state1, cost2, heuristic1);
    // assertFalse(test1.equals(test3));
    // Nodes are different in state ==> different
    GlcNode test4 = new GlcNode(null, state2, cost1, heuristic1);
    // assertFalse(test1.equals(test4));
  }
}
