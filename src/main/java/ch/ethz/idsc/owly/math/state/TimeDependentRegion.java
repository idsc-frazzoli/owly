// code by jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.owly.math.region.Region;

/** StateTimeRegion that depends on time */
public final class TimeDependentRegion implements StateTimeRegion {
  private final Region region;

  /** @param region */
  public TimeDependentRegion(Region region) {
    this.region = region;
  }

  /** @param StateTime of point to check
   * @return true if stateTime is member/part of/inside region */
  @Override
  public boolean isMember(StateTime stateTime) {
    return region.isMember(stateTime.joined());
  }
}
