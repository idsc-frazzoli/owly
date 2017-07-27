// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

public abstract class StandardTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  private final StateTimeRegion stateTimeRegion;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  public StandardTrajectoryRegionQuery(StateTimeRegion stateTimeRegion, StateTimeRegionCallback stateTimeRegionCallback) {
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
