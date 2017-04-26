// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.StateTimeRegion;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;

public class SimpleTrajectoryRegionQuery implements TrajectoryRegionQuery {
  private final StateTimeRegion stateTimeRegion;

  public SimpleTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    this.stateTimeRegion = stateTimeRegion;
  }

  @Override
  public final int firstMember(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime))
        return index;
    }
    return NOMATCH;
  }
}
