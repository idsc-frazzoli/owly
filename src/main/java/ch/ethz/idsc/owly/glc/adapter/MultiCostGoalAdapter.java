// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Max;

/** combines multiple cost functions */
public class MultiCostGoalAdapter implements GoalInterface {
  private final TrajectoryRegionQuery trajectoryRegionQuery;
  private final List<CostFunction> list;

  public MultiCostGoalAdapter(TrajectoryRegionQuery trajectoryRegionQuery, List<CostFunction> list) {
    this.trajectoryRegionQuery = trajectoryRegionQuery;
    this.list = list;
  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return list.stream() //
        .map(costFunction -> costFunction.costIncrement(glcNode, trajectory, flow)) //
        .reduce(Scalar::add).get();
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor x) {
    return list.stream() //
        .map(costFunction -> costFunction.minCostToGoal(x)) //
        .reduce(Max::of).get();
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
