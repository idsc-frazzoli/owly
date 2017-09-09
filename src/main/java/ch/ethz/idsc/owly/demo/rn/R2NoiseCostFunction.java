// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.Trajectories;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.noise.ContinuousNoise;
import ch.ethz.idsc.owly.math.noise.ContinuousNoiseUtils;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** typically superimposed on min dist cost function */
public class R2NoiseCostFunction implements CostFunction {
  private static final ContinuousNoise CONTINUOUS_NOISE = ContinuousNoiseUtils.wrap2D(SimplexContinuousNoise.FUNCTION);

  // ---
  public R2NoiseCostFunction() {
  }

  @Override
  public Scalar minCostToGoal(Tensor tensor) {
    return RealScalar.ZERO;
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    Tensor cost = Tensor.of(trajectory.stream().map(StateTime::state).map(R2NoiseCostFunction::pointCost));
    Tensor dts = Trajectories.deltaTimes(glcNode, trajectory);
    return cost.dot(dts).Get();
  }

  /** @param tensor vector with at least 2 entries
   * @return value in the interval [0, 2] */
  private static Scalar pointCost(Tensor tensor) {
    return RealScalar.ONE.add(CONTINUOUS_NOISE.apply(tensor)).divide(RealScalar.of(2));
  }
}
