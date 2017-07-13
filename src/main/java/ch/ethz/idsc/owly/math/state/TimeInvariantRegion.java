// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.owly.math.region.Region;

/** StateTimeRegion, which is independent of time,
 * or with infinite expansion on the time-axis */
public final class TimeInvariantRegion implements StateTimeRegion {
  private final Region region;

  public TimeInvariantRegion(Region region) {
    this.region = region;
  }

  @Override
  /** @param StateTime of point to check
   * @return true if stateTime is member/part of/inside region */
  public boolean isMember(StateTime stateTime) {
    return region.isMember(stateTime.x());
  }
}
