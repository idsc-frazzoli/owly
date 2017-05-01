// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

public interface StateTimeRegion {
  /** @param stateTime
   * @return */
  boolean isMember(StateTime stateTime);
}
