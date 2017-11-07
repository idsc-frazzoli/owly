// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

import ch.ethz.idsc.owly.math.region.Region;

public abstract class StandardTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  private final Region<StateTime> stateTimeRegion;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  public StandardTrajectoryRegionQuery(Region<StateTime> stateTimeRegion, StateTimeRegionCallback stateTimeRegionCallback) {
    this.stateTimeRegion = stateTimeRegion;
    this.stateTimeRegionCallback = stateTimeRegionCallback;
  }

  @Override
  public final int firstMember(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime)) {
        stateTimeRegionCallback.notify_isMember(stateTime);
        return index;
      }
    }
    return NOMATCH;
  }

  public StateTimeRegionCallback getStateTimeRegionCallback() {
    return stateTimeRegionCallback;
  }
}
