// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

public class DeltaGoalManager extends SimpleTrajectoryRegionQuery implements CostFunction {
  final Tensor center;
  final Scalar radius;

  public DeltaGoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    this.center = center;
    if (!radius.Get(0).equals(radius.Get(1)))
      throw new RuntimeException(); // x-y radius have to be equal
    this.radius = radius.Get(0);
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return RealScalar.of(trajectory.size());
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return ZeroScalar.get();
  }
}
