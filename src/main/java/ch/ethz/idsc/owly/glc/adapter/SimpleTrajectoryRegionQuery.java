// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.data.FloorMap;
import ch.ethz.idsc.owly.math.state.AbstractTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.Tensors;

public class SimpleTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  private final StateTimeRegion stateTimeRegion;
  private final FloorMap<StateTime> discoveredMembers = new FloorMap<>(Tensors.vector(10, 10));

  public SimpleTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    this.stateTimeRegion = stateTimeRegion;
  }

  @Override
  public final int firstMember(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime)) {
        discoveredMembers.put(stateTime.x(), stateTime);
        return index;
      }
    }
    return NOMATCH;
  }

  public Collection<StateTime> getDiscoveredMembers() {
    return discoveredMembers.values();
  }
}
