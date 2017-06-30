// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

/** instance encodes an empty trajectory region
 * 
 * all intersection queries with a trajectory return: "empty intersection" */
public enum EmptyTrajectoryRegionQuery implements TrajectoryRegionQuery {
  INSTANCE;
  // ---
  @Override
  public int firstMember(List<StateTime> trajectory) {
    return NOMATCH;
  }

  @Override
  public boolean isDisjoint(List<StateTime> trajectory) {
    return true;
  }
}
