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
import ch.ethz.idsc.tensor.sca.Ramp;

/** the cost increment may be zero
 * therefore, min cost to goal also is zero
 * 
 * typically superimposed on min dist cost function */
public class R2NoiseCostFunction implements CostFunction {
  private static final ContinuousNoise CONTINUOUS_NOISE = //
      ContinuousNoiseUtils.wrap2D(SimplexContinuousNoise.FUNCTION);
  // ---
  private final Scalar treshold;

  public R2NoiseCostFunction(Scalar treshold) {
    this.treshold = treshold;
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    return RealScalar.ZERO;
  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    Tensor cost = Tensor.of(trajectory.stream().map(StateTime::state).map(this::pointCost));
    Tensor dts = Trajectories.deltaTimes(glcNode, trajectory);
    return cost.dot(dts).Get();
  }

  /** @param tensor vector with at least 2 entries
   * @return value in the interval [0, 2] */
  private Scalar pointCost(Tensor tensor) {
    return Ramp.of(CONTINUOUS_NOISE.apply(tensor).subtract(treshold));
  }
}
