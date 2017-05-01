// code by bapaden and jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class StateTime {
  public final Tensor x;
  public final Scalar time;

  public StateTime(Tensor x, Scalar time) {
    this.x = x;
    this.time = time;
  }

  @Override
  public String toString() { // TODO this function should have a different name and not override toString
    return String.format("t=%s  x=%s", time, x.toString());
  }
}
