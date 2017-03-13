// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.tensor.Tensor;

public class MinTimeCost implements CostFunction {
  @Override
  public final double cost(Trajectory trajectory, Tensor u) {
    return trajectory.getDuration();
  }

  @Override
  public final double getLipschitz() {
    return 0; // TODO really 0 !?
  }
}
