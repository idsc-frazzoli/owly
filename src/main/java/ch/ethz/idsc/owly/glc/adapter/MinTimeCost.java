// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.util.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.ZeroScalar;

public class MinTimeCost implements CostFunction {
  @Override
  public final Scalar cost(Trajectory trajectory, Flow u) {
    return trajectory.getDuration();
  }

  @Override
  public final Scalar getLipschitz() {
    return ZeroScalar.get(); // TODO really 0 !?
  }
}
