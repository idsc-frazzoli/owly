// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class StateTime {
  public final Tensor tensor;
  public final Scalar time;

  public StateTime(Tensor tensor, Scalar time) {
    this.tensor = tensor;
    this.time = time;
  }

  @Override
  public String toString() {
    return String.format("t=%s  x=%s", time, tensor.toString());
  }
}
