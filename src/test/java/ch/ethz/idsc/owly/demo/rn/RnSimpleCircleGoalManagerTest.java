// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Collections;

import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class RnSimpleCircleGoalManagerTest extends TestCase {
  public void testMinCostToGoal1() {
    RnSimpleCircleHeuristicGoalManager rnGoal = new RnSimpleCircleHeuristicGoalManager(Tensors.vector(5, 0), RealScalar.of(2));
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(2, 0)), RealScalar.ONE);
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(3, 0)), RealScalar.ZERO);
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(4, 0)), RealScalar.ZERO);
  }

  public void testMinCostToGoal2() {
    RnSimpleCircleGoalManager rnGoal = new RnSimpleCircleGoalManager(Tensors.vector(5, 0), RealScalar.of(2));
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(2, 0)), RealScalar.ZERO);
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(3, 0)), RealScalar.ZERO);
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(4, 0)), RealScalar.ZERO);
  }

  public void testCostIncrement1() {
    RnSimpleCircleHeuristicGoalManager rnGoal = new RnSimpleCircleHeuristicGoalManager(Tensors.vector(5, 0), RealScalar.of(2));
    Scalar incr = rnGoal.costIncrement( //
        new StateTime(Tensors.vector(2, 2), RealScalar.ZERO), //
        Collections.singletonList(new StateTime(Tensors.vector(10, 2), RealScalar.ZERO)), null);
    assertEquals(incr, RealScalar.of(8));
  }

  public void testCostIncrement2() {
    RnSimpleCircleGoalManager rnGoal = new RnSimpleCircleGoalManager(Tensors.vector(5, 0), RealScalar.of(2));
    Scalar incr = rnGoal.costIncrement( //
        new StateTime(Tensors.vector(2, 2), RealScalar.ZERO), //
        Collections.singletonList(new StateTime(Tensors.vector(10, 2), RealScalar.ZERO)), null);
    assertEquals(incr, RealScalar.of(8));
  }

  public void testNoHeuristic1() {
    RnSimpleCircleGoalManager rnGoal = new RnSimpleCircleGoalManager(Tensors.vector(5, 0), RealScalar.of(2));
    assertFalse(HeuristicQ.of(rnGoal));
  }

  public void testNoHeuristic2() {
    RnSimpleCircleHeuristicGoalManager rnGoal = new RnSimpleCircleHeuristicGoalManager(Tensors.vector(5, 0), RealScalar.of(2));
    assertTrue(HeuristicQ.of(rnGoal));
  }
}
