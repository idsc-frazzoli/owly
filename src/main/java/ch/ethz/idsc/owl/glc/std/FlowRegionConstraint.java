// code by jph and ynager
package ch.ethz.idsc.owl.glc.std;

import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.std.PlannerConstraint;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.StateTimeRegionCallback;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;

/** Implements a @PlannerConstraint to define and check flow constraints in certain regions.
 * non-empty intersection of the trajectory with the StateTime region and the flow region
 * represents a constraint violation.
 * @param StateTime region where flow constraint should be enforced
 * @param Flow Region defining the flow constraint.
 * @param StateTimeRegionCallback function */
public class FlowRegionConstraint implements PlannerConstraint {
  private final Region<StateTime> stregion;
  private final Region<Tensor> flowregion;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  public FlowRegionConstraint(Region<StateTime> stregion, Region<Tensor> flowregion, StateTimeRegionCallback stateTimeRegionCallback) {
    this.stregion = stregion;
    this.flowregion = flowregion;
    this.stateTimeRegionCallback = stateTimeRegionCallback;
  }

  public final Optional<StateTime> firstMember(List<StateTime> trajectory, Flow flow) {
    for (StateTime stateTime : trajectory)
      if (stregion.isMember(stateTime) && flowregion.isMember(flow.getU())) {
        return Optional.of(stateTime);
      }
    return Optional.empty();
  }

  public final boolean isMember(StateTime stateTime, Flow flow) {
    return stregion.isMember(stateTime) && flowregion.isMember(flow.getU());
  }

  @Override
  public boolean isSatisfied(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return !firstMember(trajectory, flow).isPresent();
  }
}
