// code by jl
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum time */
public class RnTrajectoryGoalManager extends TrajectoryGoalManager {
  private final List<StateTime> heuristicTrajectory;
  private final Scalar radius;

  public RnTrajectoryGoalManager(List<Region> goalRegions, List<StateTime> heuristicTrajectory, Tensor radius) {
    super(goalRegions);
    this.heuristicTrajectory = heuristicTrajectory;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(Norm._2.of(x.subtract(StateTimeTrajectories.getLast(heuristicTrajectory).state())).subtract(radius)//
        .divide(RealScalar.ONE)); // divide by maximum "speed"
  }

  @Override
  public boolean hasHeuristic() {
    return true;
  }
}
