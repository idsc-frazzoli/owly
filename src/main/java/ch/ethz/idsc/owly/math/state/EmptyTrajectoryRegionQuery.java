// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import java.util.List;

public final class EmptyTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  @Override
  public int firstMember(List<StateTime> trajectory) {
    return NOMATCH;
  }
}