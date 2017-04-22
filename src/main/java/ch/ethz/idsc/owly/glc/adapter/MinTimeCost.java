// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.ZeroScalar;

public final class MinTimeCost implements CostFunction {
  @Override
  public Scalar cost(Trajectory trajectory, Flow u) {
    return trajectory.getDuration();
  }
}
