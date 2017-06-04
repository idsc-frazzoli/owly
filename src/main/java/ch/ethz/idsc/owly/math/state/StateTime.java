// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.io.Serializable;
import java.util.Objects;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** StateTime is immutable, contents of instance do not change after construction */
public final class StateTime implements Serializable {
  private final Tensor x;
  private final Scalar time;

  /** @param x the state
   * @param time the time of the state */
  public StateTime(Tensor x, Scalar time) {
    this.x = x.unmodifiable();
    this.time = time;
  }

  /** @return the state */
  public Tensor x() {
    return x;
  }

  public Scalar time() {
    return time;
  }

  public String info() {
    return String.format("t=%s  x=%s", time, x.toString());
  }

  @Override // from Object
  public int hashCode() {
    return Objects.hash(x, time);
  }

  @Override // from Object
  public boolean equals(Object object) {
    if (object instanceof StateTime) {
      StateTime stateTime = (StateTime) object;
      return x.equals(stateTime.x) && time.equals(stateTime.time);
    }
    return false;
  }
}
