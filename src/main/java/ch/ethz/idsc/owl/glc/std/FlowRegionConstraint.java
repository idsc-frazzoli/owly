// code by jph and ynager
package ch.ethz.idsc.owl.glc.std;

import java.io.Serializable;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.Region;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.StateTimeRegionCallback;
import ch.ethz.idsc.tensor.Tensor;

/** Implements a @PlannerConstraint to define and check flow constraints in certain regions.
 * non-empty intersection of the trajectory with the StateTime region and the flow region
 * represents a constraint violation. */
public class FlowRegionConstraint implements PlannerConstraint, Serializable {
  private final Region<StateTime> stregion;
  private final Region<Tensor> flowregion;
  private final StateTimeRegionCallback stateTimeRegionCallback;

  /** @param StateTime region where flow constraint should be enforced
   * @param Flow Region defining the flow constraint.
   * @param StateTimeRegionCallback function */
  public FlowRegionConstraint(Region<StateTime> stregion, Region<Tensor> flowregion, StateTimeRegionCallback stateTimeRegionCallback) {
    this.stregion = stregion;
    this.flowregion = flowregion;
    this.stateTimeRegionCallback = stateTimeRegionCallback;
  }

  private Optional<StateTime> firstMember(List<StateTime> trajectory, Flow flow) {
    Tensor u = flow.getU();
    return trajectory.stream().filter(stateTime -> stregion.isMember(stateTime) && flowregion.isMember(u)).findFirst();
    // for (StateTime stateTime : trajectory)
    // if (stregion.isMember(stateTime) && flowregion.isMember(flow.getU())) {
    // return Optional.of(stateTime);
    // }
    // return Optional.empty();
  }

  // private boolean isMember(StateTime stateTime, Flow flow) {
  // return stregion.isMember(stateTime) && flowregion.isMember(flow.getU());
  // }
  @Override
  public boolean isSatisfied(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return !firstMember(trajectory, flow).isPresent();
  }
}
