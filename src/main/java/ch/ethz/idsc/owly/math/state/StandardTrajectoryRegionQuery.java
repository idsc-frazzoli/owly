// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owly.math.region.Region;

public abstract class StandardTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  private final Region<StateTime> region;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  public StandardTrajectoryRegionQuery(Region<StateTime> region, StateTimeRegionCallback stateTimeRegionCallback) {
    this.region = region;
    this.stateTimeRegionCallback = stateTimeRegionCallback;
  }

  @Override // from TrajectoryRegionQuery
  public final Optional<StateTime> firstMember(List<StateTime> trajectory) {
    for (StateTime stateTime : trajectory)
      if (region.isMember(stateTime)) {
        stateTimeRegionCallback.notify_isMember(stateTime);
        return Optional.of(stateTime);
      }
    return Optional.empty();
  }

  public StateTimeRegionCallback getStateTimeRegionCallback() {
    return stateTimeRegionCallback;
  }
}
