// code by jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;

/** StateTimeRegion that depends on time */
public final class TimeDependentRegion implements Region<StateTime> {
  private final Region<Tensor> region;

  /** @param region */
  public TimeDependentRegion(Region<Tensor> region) {
    this.region = region;
  }

  /** @param StateTime of point to check
   * @return true if stateTime is member/part of/inside region */
  @Override
  public boolean isMember(StateTime stateTime) {
    return region.isMember(stateTime.joined());
  }
}
