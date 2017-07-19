// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import ch.ethz.idsc.owly.data.SerialHashMap;
import ch.ethz.idsc.owly.math.state.AbstractTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.io.Serialization;

public class GoalTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  private final StateTimeRegion stateTimeRegion;
  // Key: endPoint of Trajectory (where Node is), Value: where the goal was found
  private SerialHashMap<StateTime, StateTime> discoveredGoalMembers = new SerialHashMap<>();

  public GoalTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    this.stateTimeRegion = stateTimeRegion;
  }

  public GoalTrajectoryRegionQuery(GoalTrajectoryRegionQuery goalTrajectoryRegionQuery) {
    try {
      discoveredGoalMembers = Serialization.copy(goalTrajectoryRegionQuery.discoveredGoalMembers);
    } catch (Exception e) {
      e.printStackTrace();
    }
    this.stateTimeRegion = goalTrajectoryRegionQuery.stateTimeRegion;
  }

  @Override
  public final int firstMember(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime)) {
        discoveredGoalMembers.put(trajectory.get(trajectory.size() - 1), stateTime); // add first member and end of trajectory (Node)
        return index;
      }
    }
    return NOMATCH;
  }

  public Collection<StateTime> getAllDiscoveredMembers() {
    return Collections.unmodifiableCollection(discoveredGoalMembers.values());
  }

  public Collection<StateTime> getAllDiscoveredMembersNodesStateTime() {
    return Collections.unmodifiableCollection(discoveredGoalMembers.keySet());
  }

  public Map<StateTime, StateTime> getMap() {
    return Collections.unmodifiableMap(discoveredGoalMembers);
  }
}
