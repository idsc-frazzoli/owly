// code by bapaden and jph
package ch.ethz.idsc.owly.math;

public interface StateTimeRegion {
  /** @param stateTime
   * @return */
  boolean isMember(StateTime stateTime);
}
