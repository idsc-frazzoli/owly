// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.Scalar;

public final class MinTimeCost implements CostFunction {
  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow u) {
    return Trajectory.getLast(trajectory).time.subtract(from.time);
  }
}
