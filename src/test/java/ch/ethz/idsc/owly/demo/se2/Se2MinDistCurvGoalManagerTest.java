// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;

import ch.ethz.idsc.owly.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Se2MinDistCurvGoalManagerTest extends TestCase {
  public void testMinDist() {
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.1), DoubleScalar.of(0.1), RealScalar.of(Math.PI * 0.1));
    Collection<Flow> controls = Se2Controls.createControlsForwardAndReverse(RealScalar.of(0.2), 10);
    Se2MinDistCurvGoalManager se2MinDistGoalManager = new Se2MinDistCurvGoalManager(//
        Tensors.vector(0, 0, Math.PI), radiusVector, controls);
    assertTrue(HeuristicQ.of(se2MinDistGoalManager.getGoalInterface()));
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

  public void testWrapExt() {
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.1), DoubleScalar.of(0.1), RealScalar.of(Math.PI * 0.1));
    Collection<Flow> controls = Se2Controls.createControlsForwardAndReverse(RealScalar.of(0.2), 10);
    Se2MinDistCurvGoalManager se2MinDistGoalManager = new Se2MinDistCurvGoalManager(//
        Tensors.vector(0, 0, Math.PI), radiusVector, controls);
    CoordinateWrap se2Wrap = new Se2Wrap(Tensors.vector(1, 1, 1));
    Se2WrapGoalManagerExt se2WrapGoalManagerExt = new Se2WrapGoalManagerExt(se2Wrap, se2MinDistGoalManager);
    assertTrue(HeuristicQ.of(se2WrapGoalManagerExt.getGoalInterface()));
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
