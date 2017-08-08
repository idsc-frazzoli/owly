// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.owly.math.region.Region;

/** StateTimeRegion, which is independent of time.
 * membership is determined in state space regardless of time.
 * membership is extended indefinitely along the time-axis */
public final class RxtTimeInvariantRegion implements StateTimeRegion {
  private final Region region;

  public RxtTimeInvariantRegion(Region region) {
    this.region = region;
  }

  /** @param StateTime of point to check
   * @return true if stateTime is member/part of/inside region */
  @Override
  public boolean isMember(StateTime stateTime) {
    int toIndex = stateTime.state().length() - 1;
    return region.isMember(stateTime.state().extract(0, toIndex));
  }
}
