// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectories;
import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

public class MinTimeCost implements CostFunction {
  public static Scalar timeIncrement(StateTime from, List<StateTime> trajectory) {
    Scalar dt = Trajectories.getLast(trajectory).time.subtract(from.time);
    if (Scalars.lessEquals(dt, ZeroScalar.get()))
      throw new RuntimeException();
    return dt;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return timeIncrement(from, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return ZeroScalar.get();
  }
}
