// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.io.Serializable;

//TODO JAN: double with Regionclass. Here is a chaos between regions
public interface StateTimeRegion extends Serializable {
  /** @param stateTime
   * @return */
  boolean isMember(StateTime stateTime);
}
