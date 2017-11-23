// code by bapaden and jph
package ch.ethz.idsc.owl.math.state;

import java.util.List;
import java.util.Optional;

/** instance encodes an empty trajectory region
 * 
 * all intersection queries with a trajectory return: "empty intersection" */
public enum EmptyTrajectoryRegionQuery implements TrajectoryRegionQuery {
  INSTANCE;
  // ---
  @Override
  public Optional<StateTime> firstMember(List<StateTime> trajectory) {
    return Optional.empty();
  }

  // @Override
  // public boolean isDisjoint(List<StateTime> trajectory) {
  // return true;
  // }
  @Override // from TrajectoryRegionQuery
  public final boolean isMember(StateTime stateTime) {
    return false;
  }
}
