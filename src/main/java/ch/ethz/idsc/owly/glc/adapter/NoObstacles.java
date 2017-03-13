// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;

public class NoObstacles implements TrajectoryRegionQuery {
  @Override
  public int firstMember(Trajectory trajectory) {
    return NOMATCH;
  }
}
