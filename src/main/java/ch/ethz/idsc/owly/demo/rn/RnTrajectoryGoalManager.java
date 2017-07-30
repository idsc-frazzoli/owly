// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TrajectoryGoalManager;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum path length */
public class RnTrajectoryGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface, TrajectoryGoalManager {
  private final Tensor center;
  private final Scalar radius;

  // TODO JONAS change heuristic center to different way
  public RnTrajectoryGoalManager(List<Region> goalRegions, Tensor heuristicCenter, Scalar radius) {
    super(new TimeInvariantRegion(RegionUnion.wrap(goalRegions)));
    this.center = heuristicCenter;
    this.radius = radius;
  }

  public RnTrajectoryGoalManager(Region region, Tensor heuristicCenter, Scalar radius) {
    super(new TimeInvariantRegion(region));
    this.center = heuristicCenter;
    this.radius = radius;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.state().subtract(Trajectories.getLast(trajectory).state()));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // return RealScalar.ZERO;
    return Ramp.of(Norm._2.of(x.subtract(center)).subtract(radius));
  }
}
