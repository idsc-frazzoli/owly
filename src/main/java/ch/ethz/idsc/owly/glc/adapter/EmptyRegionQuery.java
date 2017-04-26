// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;

public final class EmptyRegionQuery implements TrajectoryRegionQuery {
  @Override
  public int firstMember(List<StateTime> trajectory) {
    return NOMATCH;
  }
}
