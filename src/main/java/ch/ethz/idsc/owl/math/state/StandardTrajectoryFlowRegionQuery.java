// code by jph and ynager
package ch.ethz.idsc.owl.math.state;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;

public class StandardTrajectoryFlowRegionQuery implements TrajectoryFlowRegionQuery {
  private final Region<StateTime> stregion;
  private final Region<Tensor> flowregion;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  public StandardTrajectoryFlowRegionQuery(Region<StateTime> stregion, Region<Tensor> flowregion, StateTimeRegionCallback stateTimeRegionCallback) {
    this.stregion = stregion;
    this.flowregion = flowregion;
    this.stateTimeRegionCallback = stateTimeRegionCallback;
  }

  @Override // from TrajectoryFlowRegionQuery
  public final Optional<StateTime> firstMember(List<StateTime> trajectory, Flow flow) {
    for (StateTime stateTime : trajectory)
      if (stregion.isMember(stateTime) && flowregion.isMember(flow.getU())) {
        return Optional.of(stateTime);
      }
    return Optional.empty();
  }

  @Override // from TrajectoryFlowRegionQuery
  public final boolean isMember(StateTime stateTime, Flow flow) {
    return stregion.isMember(stateTime) && flowregion.isMember(flow.getU());
  }
}
