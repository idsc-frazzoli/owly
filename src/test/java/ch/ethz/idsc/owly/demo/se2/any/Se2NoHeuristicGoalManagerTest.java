// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import ch.ethz.idsc.owl.glc.adapter.HeuristicQ;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Se2NoHeuristicGoalManagerTest extends TestCase {
  public void testDefault() {
    Se2NoHeuristicGoalManager se2DefaultGoalManager = new Se2NoHeuristicGoalManager(//
        Tensors.vector(0, 0, Math.PI), //
        Tensors.vector(0.1, 0.1, 0.1 * Math.PI));
    assertFalse(HeuristicQ.of(se2DefaultGoalManager.getGoalInterface()));
    assertEquals(se2DefaultGoalManager.minCostToGoal(Tensors.vector(0, 0, 1.1 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2DefaultGoalManager.minCostToGoal(Tensors.vector(2, 0, 2 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2DefaultGoalManager.minCostToGoal(Tensors.vector(0, 2, -2 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2DefaultGoalManager.minCostToGoal(Tensors.vector(0, 0, 0 * Math.PI)), RealScalar.ZERO);
    // --
    assertTrue(se2DefaultGoalManager.isMember(Tensors.vector(0.05, -0.05, Math.PI * 0.95)));
    assertTrue(se2DefaultGoalManager.isMember(Tensors.vector(0.0, 0.0, Math.PI)));
    assertTrue(se2DefaultGoalManager.isMember(Tensors.vector(0.0, 0.0, 3 * Math.PI)));
    // --
    assertFalse(se2DefaultGoalManager.isMember(Tensors.vector(2, 2, Math.PI)));
    assertFalse(se2DefaultGoalManager.isMember(Tensors.vector(0, 0, 0)));
  }
}
