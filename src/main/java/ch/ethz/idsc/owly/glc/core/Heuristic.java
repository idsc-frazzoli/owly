// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Tensor;

public interface Heuristic {
  public double costToGo(Tensor tensor);
}
