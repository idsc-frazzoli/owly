// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import ch.ethz.idsc.owly.math.state.AbstractTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.io.Serialization;

public class GoalTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  protected final StateTimeRegion stateTimeRegion;
  // Key StateTime of where Goal was found, Value: endpoint of traj /EndNode of traj
  // TODO JAN: you said I can remove Hash, but gives error, due to Serialization
  private HashMap<StateTime, StateTime> discoveredGoalMembers = new HashMap<>();

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

  // TODO JONAS to be confirmed, that if this function is called in getfurtheststate, that value stays the same
  @Override
  public final int firstMember(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime)) {
        StateTime test = discoveredGoalMembers.put(stateTime, trajectory.get(trajectory.size() - 1));
        if (test != null)
          if (test != trajectory.get(trajectory.size() - 1))
            throw new RuntimeException();
        return index;
      }
    }
    return NOMATCH;
  }

  // TODO JAN: this class is only needed for this function,
  // could also copy the instance of SimpletrajectoryRQ, in getFurthestGoalState
  /** The same functionality as firstMember, but does not save the found state
   * @param trajectory
   * @return */
  @Deprecated
  public final int firstMemberCheck(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime)) {
        return index;
      }
    }
    return NOMATCH;
  }

  /** @param goalState: the State, which was found in the Goal
   * @return endState: the State of the Node at the end of his trajectory */
  public final StateTime getEndNode(StateTime goalState) {
    return discoveredGoalMembers.get(goalState);
  }

  public final Collection<StateTime> getAllDiscoveredMembersStateTimeInGoal() {
    return Collections.unmodifiableCollection(discoveredGoalMembers.keySet());
  }

  public final Collection<StateTime> getAllDiscoveredMembersEndNodesStateTime() {
    return Collections.unmodifiableCollection(discoveredGoalMembers.values());
  }
}
