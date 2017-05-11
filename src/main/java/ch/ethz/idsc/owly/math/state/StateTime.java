// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** StateTime is immutable, contents of instance do not change after construction */
public final class StateTime {
  private final Tensor x;
  private final Scalar time;

  /** @param x
   * @param time */
  public StateTime(Tensor x, Scalar time) {
    this.x = x.unmodifiable();
    this.time = time;
  }

  /** @return */
  public Tensor x() {
    return x;
  }

  public Scalar time() {
    return time;
  }

  public String info() {
    return String.format("t=%s  x=%s", time, x.toString());
  }
}
