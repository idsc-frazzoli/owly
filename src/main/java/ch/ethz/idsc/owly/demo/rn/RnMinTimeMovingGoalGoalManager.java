// code by jph and jl
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** objective is minimum time.
 * 
 * <p>The distance cost function is suitable for entities that are capable
 * and may need to linger in one spot (u == {0, 0}) because in that case
 * the cost == distance traveled evaluates a non-zero, positive value. */
public class RnMinTimeMovingGoalGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  /** @param StateTime Region */
  public RnMinTimeMovingGoalGoalManager(Region<StateTime> region) {
    super(region);
  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO; // TODO JONAS find heuristic: shortest path to current goal position?
  }
}