package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;

public class TimeVariantRegion implements StateTimeRegion {
  // TODO JAN, JONAS do not understand how regions are connected
  @Override
  public boolean isMember(StateTime stateTime) {
    return false;
  }
}
