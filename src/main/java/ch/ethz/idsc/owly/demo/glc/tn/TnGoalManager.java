// code by jph
package ch.ethz.idsc.owly.demo.glc.tn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** goal region is spherical
 * 
 * objective is minimum path length */
class TnGoalManager extends SimpleTrajectoryRegionQuery implements CostFunction {
  private final Tensor center;
  private final Scalar radius;
  private final TnWrap tnWarp;

  public TnGoalManager(Tensor center, Scalar radius, TnWrap tnWarp) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, Array.of(l -> radius, center.length()))));
    this.center = center;
    this.radius = radius;
    this.tnWarp = tnWarp;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.x().subtract(Trajectories.getLast(trajectory).x()));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(tnWarp.distance(x, center).subtract(radius));
  }
}
