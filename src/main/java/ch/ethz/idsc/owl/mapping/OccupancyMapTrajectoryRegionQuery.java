// code by ynager
package ch.ethz.idsc.owl.mapping;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.math.state.AbstractTrajectoryRegionQuery;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.StateTimeRegionCallback;

public class OccupancyMapTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  private OccupancyMap2d occupancyMap;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  public OccupancyMapTrajectoryRegionQuery(OccupancyMap2d occupancyMap, StateTimeRegionCallback stateTimeRegionCallback) {
    this.occupancyMap = occupancyMap;
    this.stateTimeRegionCallback = stateTimeRegionCallback;
  }

  @Override // from TrajectoryRegionQuery
  public final Optional<StateTime> firstMember(List<StateTime> trajectory) {
    for (StateTime stateTime : trajectory)
      if (isMember(stateTime)) {
        return Optional.of(stateTime);
      }
    return Optional.empty();
  }

  @Override // from TrajectoryRegionQuery
  public final boolean isMember(StateTime stateTime) {
    return occupancyMap.isOccupied(stateTime.state().extract(0, 2));
  }

  public StateTimeRegionCallback getStateTimeRegionCallback() {
    return stateTimeRegionCallback;
  }
}
