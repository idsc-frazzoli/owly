// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.alg.Last;

/** StateTimeRegion, which is independent of time.
 * membership is determined in state space regardless of time.
 * membership is extended indefinitely along the time-axis
 * 
 * implementation requires that last entry of StateTime::state is
 * identical to StateTime::time */
public final class RxtTimeInvariantRegion implements Region<StateTime> {
  private final Region<Tensor> region;

  public RxtTimeInvariantRegion(Region<Tensor> region) {
    this.region = region;
  }

  /** @param StateTime of point to check
   * @return true if stateTime is member/part of/inside region */
  @Override
  public boolean isMember(StateTime stateTime) {
    // consistency check
    if (!Last.of(stateTime.state()).equals(stateTime.time()))
      throw TensorRuntimeException.of(stateTime.state(), stateTime.time());
    // ---
    int toIndex = stateTime.state().length() - 1;
    return region.isMember(stateTime.state().extract(0, toIndex));
  }
}
