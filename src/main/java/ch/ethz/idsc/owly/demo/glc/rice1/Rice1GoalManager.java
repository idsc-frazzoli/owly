// code by jph
package ch.ethz.idsc.owly.demo.glc.rice1;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.math.StateTime;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

class Rice1GoalManager extends SimpleTrajectoryRegionQuery implements CostFunction {
  final Tensor center;
  final Tensor radius;

  public Rice1GoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    this.center = center;
    this.radius = radius;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Trajectories.timeIncrement(from, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    Scalar pc = x.Get(0);
    Scalar pd = center.Get(0);
    Scalar mindist = Ramp.function.apply(Norm._2.of(pc.subtract(pd)).subtract(radius.get(0)));
    return mindist; // .divide(1 [m/s]), since max velocity == 1 => division is obsolete
  }
}
