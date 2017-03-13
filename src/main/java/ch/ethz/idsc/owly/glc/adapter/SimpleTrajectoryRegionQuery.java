// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.StateTimeRegion;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;

public class SimpleTrajectoryRegionQuery implements TrajectoryRegionQuery {
  final StateTimeRegion stateTimeRegion;

  public SimpleTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    this.stateTimeRegion = stateTimeRegion;
  }

  @Override
  public final int firstMember(Trajectory trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime))
        return index;
    }
    return NOMATCH;
  }
}
