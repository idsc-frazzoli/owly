// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** StateTime is immutable, contents of instance do not change after construction */
public final class StateTime implements Serializable {
  private final Tensor x;
  private final Scalar time;
  private volatile int hashCode;

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

  @Override
  public boolean equals(Object obj) {
    if (obj == null)
      return false;
    // TODO implement class check, like this blocks everything
    // if (StateTime.class.isAssignableFrom(obj.getClass())) {
    // System.out.println("debug12");
    // return false;
    // }
    final StateTime other = (StateTime) obj;
    if (!this.x.equals(other.x))
      return false;
    if (!this.time.equals(other.time))
      return false;
    return true;
  }

  @Override
  public int hashCode() {
    int result = hashCode;
    if (result == 0) {
      result = 17;
      result = 31 * result + x.hashCode();
      result = 31 * result + time.hashCode();
      hashCode = result;
    }
    return result;
  }
}
