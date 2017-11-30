// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.owl.glc.adapter.HeuristicQ;
import ch.ethz.idsc.owl.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.se2.CarStandardFlows;
import ch.ethz.idsc.owly.demo.se2.Se2LateralAcceleration;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class Se2MinCurvatureGoalManagerTest extends TestCase {
  public void testMinCurvature() {
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.1), DoubleScalar.of(0.1), RealScalar.of(Math.PI * 0.1));
    Tensor goalCenter = Tensors.vector(0, 0, Math.PI);
    CarStandardFlows carConfig = new CarStandardFlows(RealScalar.ONE, Degree.of(45));
    Collection<Flow> controls = carConfig.getFlows(10); // Resolution of 10 (magic const)
    GoalInterface se2GoalManager = MultiCostGoalAdapter.of( //
        Se2MinTimeGoalManager.create(goalCenter, radiusVector, controls), //
        Arrays.asList(Se2LateralAcceleration.COSTFUNCTION));
    assertTrue(HeuristicQ.of(se2GoalManager));
    assertEquals(se2GoalManager.minCostToGoal(Tensors.vector(0, 0, 1.09 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2GoalManager.minCostToGoal(Tensors.vector(0.05, 0.05, 3 * Math.PI)), RealScalar.ZERO);
    assertEquals(se2GoalManager.minCostToGoal(Tensors.vector(1.1, 0, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2GoalManager.minCostToGoal(Tensors.vector(0, 1.1, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2GoalManager.minCostToGoal(Tensors.vector(-1.1, 0, 1.1 * Math.PI)), RealScalar.ONE);
    assertEquals(se2GoalManager.minCostToGoal(Tensors.vector(0, -1.1, 1.1 * Math.PI)), RealScalar.ONE);
    assertTrue(Scalars.lessThan(RealScalar.ZERO, //
        se2GoalManager.minCostToGoal(Tensors.vector(0, 0, 1.5 * Math.PI))));
    assertTrue(Scalars.lessThan(RealScalar.ZERO, //
        se2GoalManager.minCostToGoal(Tensors.vector(0, 0, -1.5 * Math.PI))));
    // --
    assertTrue(se2GoalManager.isMember(new StateTime(Tensors.vector(0.05, -0.05, Math.PI * 0.95), RealScalar.ZERO)));
    assertTrue(se2GoalManager.isMember(new StateTime(Tensors.vector(0.0, 0.0, Math.PI), RealScalar.ZERO)));
    assertTrue(se2GoalManager.isMember(new StateTime(Tensors.vector(0.0, 0.0, 3 * Math.PI), RealScalar.ZERO)));
    // --
    assertFalse(se2GoalManager.isMember(new StateTime(Tensors.vector(2, 2, Math.PI), RealScalar.ZERO)));
    assertFalse(se2GoalManager.isMember(new StateTime(Tensors.vector(0, 0, 0), RealScalar.ZERO)));
  }
}
