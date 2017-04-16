// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.StateTimeRegion;
import ch.ethz.idsc.owly.util.Region;

public class TimeInvariantRegion implements StateTimeRegion {
  private final Region region;

  public TimeInvariantRegion(Region region) {
    this.region = region;
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    return region.isMember(stateTime.x);
  }
}
