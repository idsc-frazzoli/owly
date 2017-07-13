// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.io.Serializable;

public interface StateTimeRegion extends Serializable {
  /** @param stateTime
   * @return */
  boolean isMember(StateTime stateTime);
}
