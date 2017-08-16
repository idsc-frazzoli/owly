// code by jph
package ch.ethz.idsc.owly.demo.rnxt.glc;

import java.util.Collections;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class R2xtEllipsoidGoalManagerTest extends TestCase {
  public void testMinCostToGoal0() {
    R2xtHeuristicEllipsoidGoalManager rnxtGoal = new R2xtHeuristicEllipsoidGoalManager(//
        Tensors.vector(5, 0, -99)//
        , Tensors.vector(2, 3, Double.POSITIVE_INFINITY));
    assertEquals(rnxtGoal.minCostToGoal(Tensors.vector(2, 0, 0)), RealScalar.ONE);
    assertEquals(rnxtGoal.minCostToGoal(Tensors.vector(3, 0, 3)), RealScalar.ZERO);
    assertEquals(rnxtGoal.minCostToGoal(Tensors.vector(4, 0, 5)), RealScalar.ZERO);
  }

  public void testMinCostToGoal1() {
    R2xtHeuristicEllipsoidGoalManager rnxtGoal = new R2xtHeuristicEllipsoidGoalManager(//
        Tensors.vector(0, 5, -99)//
        , Tensors.vector(2, 3, Double.POSITIVE_INFINITY));
    assertEquals(rnxtGoal.minCostToGoal(Tensors.vector(0, 1, 0)), RealScalar.ONE);
    assertEquals(rnxtGoal.minCostToGoal(Tensors.vector(0, 2, 3)), RealScalar.ZERO);
    assertEquals(rnxtGoal.minCostToGoal(Tensors.vector(0, 3, 0)), RealScalar.ZERO);
  }

  public void testMinCostToGoal2() {
    RnxtEllipsoidGoalManager rnGoal = new RnxtEllipsoidGoalManager(//
        Tensors.of(RealScalar.of(5), RealScalar.ZERO, DoubleScalar.POSITIVE_INFINITY)//
        , RealScalar.of(2));
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(2, 0)), RealScalar.ZERO);
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(3, 0)), RealScalar.ZERO);
    assertEquals(rnGoal.minCostToGoal(Tensors.vector(4, 0)), RealScalar.ZERO);
    // TODO JONAS test for non-zero minCostToGoal results
    // FIXME JONAS the minCostToGoal is most likely not correct: center_t=inf, radius<inf ! should give other values
  }

  public void testCostIncrement1() {
    R2xtHeuristicEllipsoidGoalManager rnxtGoal = new R2xtHeuristicEllipsoidGoalManager(//
        Tensors.of(RealScalar.of(5), RealScalar.ZERO, DoubleScalar.POSITIVE_INFINITY)//
        , RealScalar.of(2));
    Scalar incr = rnxtGoal.costIncrement( //
        new StateTime(Tensors.vector(2, 2, 0), RealScalar.ZERO), //
        Collections.singletonList(new StateTime(Tensors.vector(10, 2, 8), RealScalar.of(8))), null);
    assertEquals(incr, RealScalar.of(8));
  }

  public void testCostIncrement2() {
    RnxtEllipsoidGoalManager rnGoal = new RnxtEllipsoidGoalManager(//
        Tensors.of(RealScalar.of(5), RealScalar.ZERO, DoubleScalar.POSITIVE_INFINITY)//
        , RealScalar.of(2));
    Scalar incr = rnGoal.costIncrement( //
        new StateTime(Tensors.vector(2, 2, 0), RealScalar.ZERO), //
        Collections.singletonList(new StateTime(Tensors.vector(10, 2, 8), RealScalar.of(8))), null);
    assertEquals(incr, RealScalar.of(8));
  }
}
