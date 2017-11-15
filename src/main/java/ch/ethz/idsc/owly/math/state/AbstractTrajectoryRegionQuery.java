// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

/** straightforward implementation of function isDisjoint() */
public abstract class AbstractTrajectoryRegionQuery implements TrajectoryRegionQuery {
  @Override
  public final boolean isDisjoint(List<StateTime> trajectory) {
    return !firstMember(trajectory).isPresent();
  }
}
