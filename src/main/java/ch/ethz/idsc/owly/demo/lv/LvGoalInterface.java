// code by jph
package ch.ethz.idsc.owly.demo.lv;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class LvGoalInterface implements GoalInterface {
  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Trajectories.timeIncrement(from, trajectory);
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
