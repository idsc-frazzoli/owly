// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.tensor.Tensor;

public class ZeroHeuristic implements Heuristic {
  @Override
  public final double costToGo(Tensor tensor) {
    return 0;
  }
}
