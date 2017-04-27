// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

public final class ZeroHeuristic implements Heuristic {
  @Override
  public Scalar costToGoal(Tensor x) {
    return ZeroScalar.get();
  }
}
