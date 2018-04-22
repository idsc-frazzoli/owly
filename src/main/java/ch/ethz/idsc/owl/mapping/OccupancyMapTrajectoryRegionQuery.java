// code by ynager
package ch.ethz.idsc.owl.mapping;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.StateTimeRegionCallback;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;

// TODO this class doesn't do that much specific to OccupancyMap2d... a more general wrap would be enough
public class OccupancyMapTrajectoryRegionQuery implements TrajectoryRegionQuery {
  private final OccupancyMap2d occupancyMap2d;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  public OccupancyMapTrajectoryRegionQuery(OccupancyMap2d occupancyMap, StateTimeRegionCallback stateTimeRegionCallback) {
    this.occupancyMap2d = occupancyMap;
    this.stateTimeRegionCallback = stateTimeRegionCallback;
  }

  @Override // from TrajectoryRegionQuery
  public final Optional<StateTime> firstMember(List<StateTime> trajectory) {
    return trajectory.stream().filter(this::isMember).findFirst();
  }

  @Override // from TrajectoryRegionQuery
  public final boolean isMember(StateTime stateTime) {
    return occupancyMap2d.isOccupied(stateTime.state().extract(0, 2));
  }

  public StateTimeRegionCallback getStateTimeRegionCallback() {
    return stateTimeRegionCallback;
  }
}
