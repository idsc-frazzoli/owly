// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.Region;
import ch.ethz.idsc.owly.math.StateTime;
import ch.ethz.idsc.owly.math.StateTimeRegion;

public final class TimeInvariantRegion implements StateTimeRegion {
  private final Region region;

  public TimeInvariantRegion(Region region) {
    this.region = region;
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    return region.isMember(stateTime.x);
  }
}
