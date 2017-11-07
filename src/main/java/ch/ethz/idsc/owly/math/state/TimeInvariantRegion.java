// code by bapaden and jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;

/** StateTimeRegion, which is independent of time.
 * membership is determined in state space regardless of time.
 * membership is extended indefinitely along the time-axis */
public final class TimeInvariantRegion implements Region<StateTime> {
  private final Region<Tensor> region;

  public TimeInvariantRegion(Region<Tensor> region) {
    this.region = region;
  }

  /** @param StateTime of point to check
   * @return true if stateTime is member/part of/inside region */
  @Override
  public boolean isMember(StateTime stateTime) {
    return region.isMember(stateTime.state());
  }
}
