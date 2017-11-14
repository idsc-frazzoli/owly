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

/** class bundles the capabilities of a given
 * cost function and trajectory region query */
public class GoalAdapter implements GoalInterface {
  private final TrajectoryRegionQuery trajectoryRegionQuery;
  private final CostFunction costFunction;

  public GoalAdapter(TrajectoryRegionQuery trajectoryRegionQuery, CostFunction costFunction) {
    this.trajectoryRegionQuery = trajectoryRegionQuery;
    this.costFunction = costFunction;
  }

  @Override // from TrajectoryRegionQuery
  public int firstMember(List<StateTime> trajectory) {
    return trajectoryRegionQuery.firstMember(trajectory);
  }

  @Override // from TrajectoryRegionQuery
  public boolean isDisjoint(List<StateTime> trajectory) {
    return trajectoryRegionQuery.isDisjoint(trajectory);
  }

  @Override // from CostFunction
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    return costFunction.costIncrement(node, trajectory, flow);
  }

  @Override // from CostFunction
  public Scalar minCostToGoal(Tensor x) {
    return costFunction.minCostToGoal(x);
  }
}
