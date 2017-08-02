// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum path length */
public class RnTrajectoryGoalManager extends TrajectoryGoalManager {
  private final List<StateTime> heuristicTrajectory;
  private final Scalar radius;

  // TODO JONAS change heuristic center to different way
  public RnTrajectoryGoalManager(List<Region> goalRegions, List<StateTime> heuristicTrajectory, Tensor radius) {
    super(goalRegions);
    this.heuristicTrajectory = heuristicTrajectory;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.state().subtract(StateTimeTrajectories.getLast(trajectory).state()));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // return RealScalar.ZERO;
    return Ramp.of(Norm._2.of(x.subtract(StateTimeTrajectories.getLast(heuristicTrajectory).state())).subtract(radius));
  }
}
