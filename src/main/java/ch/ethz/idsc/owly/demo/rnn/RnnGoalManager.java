// code by jph
package ch.ethz.idsc.owly.demo.rnn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.noise.ContinuousNoise;
import ch.ethz.idsc.owly.math.noise.ContinuousNoiseUtils;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;

/** objective is minimum path length */
class RnnGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  @SuppressWarnings("unused")
  private final Tensor center;
  @SuppressWarnings("unused")
  private final Scalar radius;
  private final ContinuousNoise continuousNoise;

  public RnnGoalManager(Tensor center, Scalar radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, Array.of(l -> radius, center.length()))));
    this.center = center;
    this.radius = radius;
    continuousNoise = ContinuousNoiseUtils.wrap(SimplexContinuousNoise.FUNCTION);
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    Scalar sum = trajectory.stream().map(StateTime::state).map(continuousNoise).reduce(Scalar::add).get();
    sum = sum.add(RealScalar.of(trajectory.size()));
    if (Scalars.lessThan(sum, RealScalar.ZERO))
      throw new RuntimeException();
    return sum;
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }
}
