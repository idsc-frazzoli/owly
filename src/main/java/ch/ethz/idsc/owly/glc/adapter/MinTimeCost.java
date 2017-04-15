// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class MinTimeCost implements CostFunction {
  @Override
  public final Scalar cost(Trajectory trajectory, Tensor u) {
    return RealScalar.of(trajectory.getDuration());
  }

  @Override
  public final double getLipschitz() {
    return 0; // TODO really 0 !?
  }
}
