// code by jl
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Se2GoalManagerTest extends TestCase {
  public void testDefault() {
    Se2DefaultGoalManager se2DefaultGoalManager = new Se2DefaultGoalManager(//
        Tensors.vector(0, 0), RealScalar.of(Math.PI), //
        DoubleScalar.of(.1), RealScalar.of(Math.PI * 0.1));
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

  public void testMinDist() {
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.1), DoubleScalar.of(0.1), RealScalar.of(Math.PI * 0.1));
    Se2MinDistGoalManager se2MinDistGoalManager = new Se2MinDistGoalManager(//
        Tensors.vector(0, 0, Math.PI), radiusVector);
    assertEquals(se2MinDistGoalManager.minCostToGoal(Tensors.vector(0, 0, 1.1 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2MinDistGoalManager.minCostToGoal(Tensors.vector(0.05, 0.05, 3 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2MinDistGoalManager.minCostToGoal(Tensors.vector(1.1, 0, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2MinDistGoalManager.minCostToGoal(Tensors.vector(0, 1.1, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2MinDistGoalManager.minCostToGoal(Tensors.vector(-1.1, 0, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2MinDistGoalManager.minCostToGoal(Tensors.vector(0, -1.1, 1.1 * Math.PI)), RealScalar.ONE);
    // --
    assertTrue(se2MinDistGoalManager.isMember(Tensors.vector(0.05, -0.05, Math.PI * 0.95)));
    assertTrue(se2MinDistGoalManager.isMember(Tensors.vector(0.0, 0.0, Math.PI)));
    assertTrue(se2MinDistGoalManager.isMember(Tensors.vector(0.0, 0.0, 3 * Math.PI)));
    // --
    assertFalse(se2MinDistGoalManager.isMember(Tensors.vector(2, 2, Math.PI)));
    assertFalse(se2MinDistGoalManager.isMember(Tensors.vector(0, 0, 0)));
  }

  public void testMinCurvature() {
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.1), DoubleScalar.of(0.1), RealScalar.of(Math.PI * 0.1));
    Se2MinCurvatureGoalManager se2MinCurvatureGoalManager = new Se2MinCurvatureGoalManager(//
        Tensors.vector(0, 0, Math.PI), radiusVector);
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

  public void testWrapExt() {
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.1), DoubleScalar.of(0.1), RealScalar.of(Math.PI * 0.1));
    Se2MinDistGoalManager se2MinDistGoalManager = new Se2MinDistGoalManager(//
        Tensors.vector(0, 0, Math.PI), radiusVector);
    CoordinateWrap se2Wrap = new Se2Wrap(Tensors.vector(1, 1, 1));
    Se2WrapGoalManagerExt se2WrapGoalManagerExt = new Se2WrapGoalManagerExt(se2Wrap, se2MinDistGoalManager);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(0.0, 0.0, 1.0 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(0.01, 0.01, 1.01 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(0.05, 0.05, 3 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(1.1, 0, 1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(0, 1.1, 1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(-1.1, 0, 1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(0, -1.1, 1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(0, -1.1, -3 * Math.PI)), RealScalar.ONE);
    assertEquals(se2WrapGoalManagerExt.minCostToGoal(Tensors.vector(0, -1.1, 3 * Math.PI)), RealScalar.ONE);
    // --
    assertTrue(se2WrapGoalManagerExt.isMember(Tensors.vector(0.05, -0.05, Math.PI * 0.95)));
    assertTrue(se2WrapGoalManagerExt.isMember(Tensors.vector(0.0, 0.0, Math.PI)));
    assertTrue(se2WrapGoalManagerExt.isMember(Tensors.vector(0.0, 0.0, 3 * Math.PI)));
    // --
    assertFalse(se2WrapGoalManagerExt.isMember(Tensors.vector(2, 2, Math.PI)));
    assertFalse(se2WrapGoalManagerExt.isMember(Tensors.vector(0, 0, 0)));
    assertFalse(se2WrapGoalManagerExt.isMember(Tensors.vector(0, 0, 5.5 * Math.PI)));
  }
}
