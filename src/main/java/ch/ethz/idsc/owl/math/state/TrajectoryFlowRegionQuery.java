// code by bapaden, jph and ynager
package ch.ethz.idsc.owl.math.state;

import java.io.Serializable;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.math.flow.Flow;

/** performs trajectory containment query */
public interface TrajectoryFlowRegionQuery extends Serializable {
  /** @param trajectory
   * @return first {@link StateTime} along trajectory that lies inside this region,
   * or Optional.empty() if no state-time in trajectory is member of region */
  Optional<StateTime> firstMember(List<StateTime> trajectory, Flow prevflow);

  /** @param stateTime
   * @return true if given state-time is inside region */
  boolean isMember(StateTime stateTime, Flow prevflow);
}
