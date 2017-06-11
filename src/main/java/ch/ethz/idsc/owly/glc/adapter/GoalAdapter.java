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

public class GoalAdapter implements GoalInterface {
  final CostFunction costFunction;
  final TrajectoryRegionQuery trajectoryRegionQuery;

  public GoalAdapter(CostFunction costFunction, TrajectoryRegionQuery trajectoryRegionQuery) {
    this.costFunction = costFunction;
    this.trajectoryRegionQuery = trajectoryRegionQuery;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return costFunction.costIncrement(from, trajectory, flow);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return costFunction.minCostToGoal(x);
  }

  @Override
  public int firstMember(List<StateTime> trajectory) {
    return trajectoryRegionQuery.firstMember(trajectory);
  }

  @Override
  public boolean isDisjoint(List<StateTime> trajectory) {
    return trajectoryRegionQuery.isDisjoint(trajectory);
  }
}
