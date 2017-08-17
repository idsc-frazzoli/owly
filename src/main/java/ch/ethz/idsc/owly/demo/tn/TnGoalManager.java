// code by jph
package ch.ethz.idsc.owly.demo.tn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** goal region is spherical
 * 
 * objective is minimum path length */
class TnGoalManager implements Region, CostFunction {
  private final CoordinateWrap tnWarp;
  private final Tensor center;
  private final Scalar radius;

  public TnGoalManager(CoordinateWrap tnWarp, Tensor center, Scalar radius) {
    this.tnWarp = tnWarp;
    this.center = center;
    this.radius = radius;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.state().subtract(StateTimeTrajectories.getLast(trajectory).state()));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(tnWarp.distance(x, center).subtract(radius));
  }

  @Override
  public boolean hasHeuristic() {
    return true;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return Scalars.isZero(minCostToGoal(tensor));
  }

  public GoalInterface getGoalInterface() {
    return new GoalAdapter(this, new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this)));
  }
}
