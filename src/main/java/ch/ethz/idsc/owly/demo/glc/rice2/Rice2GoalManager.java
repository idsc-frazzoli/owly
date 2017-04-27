// code by jph
package ch.ethz.idsc.owly.demo.glc.rice2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.EllipsoidRegion;
import ch.ethz.idsc.owly.glc.adapter.MinTimeCost;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.TimeInvariantRegion;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

class Rice2GoalManager extends SimpleTrajectoryRegionQuery implements CostFunction, Heuristic {
  final Tensor center;
  final Tensor radius;

  public Rice2GoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    this.center = center;
    this.radius = radius;
    // TODO assert that radius(0) == radius(1)
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return MinTimeCost.timeIncrement(from, trajectory);
  }

  @Override
  public Scalar costToGoal(Tensor x) {
    Tensor pc = x.extract(0, 2);
    Tensor pd = center.extract(0, 2);
    Scalar mindist = Ramp.function.apply(Norm._2.of(pc.subtract(pd)).subtract(radius.get(0)));
    return mindist; // .divide(1 [m/s]), since max velocity == 1 => division is obsolete
  }
}
