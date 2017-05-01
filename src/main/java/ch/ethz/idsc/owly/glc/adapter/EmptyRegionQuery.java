// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.StateTime;

public final class EmptyRegionQuery implements TrajectoryRegionQuery {
  @Override
  public int firstMember(List<StateTime> trajectory) {
    return NOMATCH;
  }
}
