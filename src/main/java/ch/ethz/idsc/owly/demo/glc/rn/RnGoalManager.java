// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum path length */
class RnGoalManager extends SimpleTrajectoryRegionQuery implements CostFunction {
  final Tensor center;
  final Scalar radius;

  public RnGoalManager(Tensor center, Scalar radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, Array.of(l -> radius, center.length()))));
    this.center = center;
    this.radius = radius;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.x.subtract(Trajectories.getLast(trajectory).x));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return (Scalar) Ramp.of(Norm._2.of(x.subtract(center)).subtract(radius));
  }
}
