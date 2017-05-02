// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

/** performs trajectory containment query */
public interface TrajectoryRegionQuery {
  static final int NOMATCH = -1;

  /** @param trajectory
   * @return index of first {@link StateTime} element of trajectory that is in region
   * or -1 if no such element exists */
  int firstMember(List<StateTime> trajectory);

  /** @param trajectory
   * @return true if no members of trajectory are in region, else false */
  boolean isDisjoint(List<StateTime> trajectory);
}
