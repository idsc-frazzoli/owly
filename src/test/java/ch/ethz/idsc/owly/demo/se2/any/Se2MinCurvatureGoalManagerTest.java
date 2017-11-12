// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Se2MinCurvatureGoalManagerTest extends TestCase {
  public void testMinCurvature() {
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.1), DoubleScalar.of(0.1), RealScalar.of(Math.PI * 0.1));
    Se2MinCurvatureGoalManager se2MinCurvatureGoalManager = new Se2MinCurvatureGoalManager(//
        Tensors.vector(0, 0, Math.PI), radiusVector);
    assertTrue(HeuristicQ.of(se2MinCurvatureGoalManager.getGoalInterface()));
    assertEquals(se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(0, 0, 1.09 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(0.05, 0.05, 3 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(1.1, 0, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(0, 1.1, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(-1.1, 0, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(0, -1.1, 1.1 * Math.PI)), RealScalar.ONE);
    assertTrue(Scalars.lessThan(RealScalar.ZERO, //
        se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(0, 0, 1.5 * Math.PI))));
    assertTrue(Scalars.lessThan(RealScalar.ZERO, //
        se2MinCurvatureGoalManager.minCostToGoal(Tensors.vector(0, 0, -1.5 * Math.PI))));
    // --
    assertTrue(se2MinCurvatureGoalManager.isMember(Tensors.vector(0.05, -0.05, Math.PI * 0.95)));
    assertTrue(se2MinCurvatureGoalManager.isMember(Tensors.vector(0.0, 0.0, Math.PI)));
    assertTrue(se2MinCurvatureGoalManager.isMember(Tensors.vector(0.0, 0.0, 3 * Math.PI)));
    // --
    assertFalse(se2MinCurvatureGoalManager.isMember(Tensors.vector(2, 2, Math.PI)));
    assertFalse(se2MinCurvatureGoalManager.isMember(Tensors.vector(0, 0, 0)));
  }
}
