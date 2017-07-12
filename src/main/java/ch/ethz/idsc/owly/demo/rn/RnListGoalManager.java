// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum path length */
public class RnListGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;

  // TODO JONAS change heuristic center to different way
  public RnListGoalManager(TimeInvariantRegion goalRegion, Tensor heuristicCenter) {
    super(goalRegion);
    center = heuristicCenter;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.x().subtract(Trajectories.getLast(trajectory).x()));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(Norm._2.of(x.subtract(center)));
  }
}
