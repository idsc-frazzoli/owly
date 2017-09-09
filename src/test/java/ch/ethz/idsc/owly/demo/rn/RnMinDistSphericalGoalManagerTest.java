// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Collections;

import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RnMinDistSphericalGoalManagerTest extends TestCase {
  public void testHeuristic() {
    GoalInterface goalInterface = //
        RnMinDistSphericalGoalManager.create(Tensors.vector(5, 0), RealScalar.of(2));
    assertTrue(HeuristicQ.of(goalInterface));
  }

  public void testMinCost() {
    GoalInterface goalInterface = //
        RnMinDistSphericalGoalManager.create(Tensors.vector(5, 3), RealScalar.of(2));
    assertEquals(goalInterface.minCostToGoal(Tensors.vector(0, 3)), RealScalar.of(3));
    assertEquals(goalInterface.minCostToGoal(Tensors.vector(5, 1)), RealScalar.of(0));
    assertEquals(goalInterface.minCostToGoal(Tensors.vector(5, 0)), RealScalar.of(1));
  }

  public void testCostIncr() {
    GlcNode glcNode = GlcNode.of(null, new StateTime(Tensors.vector(10, 3), RealScalar.ZERO), RealScalar.ZERO, RealScalar.ZERO);
    GoalInterface goalInterface = //
        RnMinDistSphericalGoalManager.create(Tensors.vector(5, 3), RealScalar.of(2));
    Scalar increment = goalInterface.costIncrement( //
        glcNode, //
        Collections.singletonList(new StateTime(Tensors.vector(13, 7), RealScalar.ZERO)), //
        null);
    assertEquals(increment, RealScalar.of(5));
  }
}
