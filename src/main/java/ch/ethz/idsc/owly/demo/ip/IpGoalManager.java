// code by jph
package ch.ethz.idsc.owly.demo.ip;

import java.util.List;

import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.BoundedBoxRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

// x == [d v a w]
class IpGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  public IpGoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new BoundedBoxRegion(center, radius)));
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }
}
