// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;

public class SimpleTrajectoryRegionQuery implements TrajectoryRegionQuery {
  private final StateTimeRegion stateTimeRegion;
  private final List<StateTime> discoveredMembers = new LinkedList<>();

  public SimpleTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    this.stateTimeRegion = stateTimeRegion;
  }

  @Override
  public final int firstMember(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime)) {
        discoveredMembers.add(stateTime); // TODO make sure that this doesn't grow indefinitely
        return index;
      }
    }
    return NOMATCH;
  }

  public Collection<StateTime> getDiscoveredMembers() {
    return Collections.unmodifiableCollection(discoveredMembers);
  }
}
