// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** represents an empty/unreachable goal region
 * 
 * cost are increments in time
 * 
 * implementation is useful to explore/search space */
public enum EmptyGoalInterface implements GoalInterface {
  INSTANCE;
  // ---
  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }

  @Override
  public int firstMember(List<StateTime> trajectory) {
    return TrajectoryRegionQuery.NOMATCH;
  }

  @Override
  public boolean isDisjoint(List<StateTime> trajectory) {
    return true;
  }
}
