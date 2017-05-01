// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

public interface TrajectoryRegionQuery {
  static final int NOMATCH = -1;

  /** @param trajectory
   * @return index of first {@link StateTime} element of trajectory that is in region
   * or -1 if no such element exists */
  int firstMember(List<StateTime> trajectory);

  default boolean isDisjoint(List<StateTime> trajectory) {
    return NOMATCH == firstMember(trajectory);
  }
}
