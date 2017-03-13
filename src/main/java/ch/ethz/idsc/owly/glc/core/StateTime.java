// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Tensor;

public class StateTime {
  public final Tensor tensor;
  public final double time;

  public StateTime(Tensor tensor, double time) {
    this.tensor = tensor;
    this.time = time;
  }

  @Override
  public String toString() {
    return String.format("t=%6.3f  x=%s", time, tensor.toString());
  }
}
