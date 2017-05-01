// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.owly.math.region.Region;

public final class TimeInvariantRegion implements StateTimeRegion {
  private final Region region;

  public TimeInvariantRegion(Region region) {
    this.region = region;
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    return region.isMember(stateTime.x());
  }
}
