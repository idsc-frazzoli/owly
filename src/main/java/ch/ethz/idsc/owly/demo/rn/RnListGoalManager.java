// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.GoalTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionUnion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

/** objective is minimum path length */
public class RnListGoalManager extends GoalTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;

  // TODO JONAS change heuristic center to different way
  public RnListGoalManager(List<Region> goalRegions, Tensor heuristicCenter) {
    super(new TimeInvariantRegion(RegionUnion.wrap(goalRegions)));
    center = heuristicCenter;
  }

  public RnListGoalManager(Region region, Tensor heuristicCenter) {
    super(new TimeInvariantRegion(region));
    center = heuristicCenter;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.x().subtract(Trajectories.getLast(trajectory).x()));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
    // return Ramp.of(Norm._2.of(x.subtract(center)));
  }
}
