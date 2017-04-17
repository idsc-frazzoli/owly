// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

public interface StateTimeRegion {
  /** @param stateTime
   * @return */
  boolean isMember(StateTime stateTime);
}
