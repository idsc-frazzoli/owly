// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** class bundles the capabilities of a given
 * cost function and trajectory region query */
public class GoalAdapter implements GoalInterface {
  private final CostFunction costFunction;
  private final TrajectoryRegionQuery trajectoryRegionQuery;

  public GoalAdapter(CostFunction costFunction, TrajectoryRegionQuery trajectoryRegionQuery) {
    this.costFunction = costFunction;
    this.trajectoryRegionQuery = trajectoryRegionQuery;
  }

  @Override // from CostFunction
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return costFunction.costIncrement(from, trajectory, flow);
  }

  @Override // from CostFunction
  public Scalar minCostToGoal(Tensor x) {
    return costFunction.minCostToGoal(x);
  }

  @Override // from TrajectoryRegionQuery
  public int firstMember(List<StateTime> trajectory) {
    return trajectoryRegionQuery.firstMember(trajectory);
  }

  @Override // from TrajectoryRegionQuery
  public boolean isDisjoint(List<StateTime> trajectory) {
    return trajectoryRegionQuery.isDisjoint(trajectory);
  }
}
