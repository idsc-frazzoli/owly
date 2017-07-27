// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;

import ch.ethz.idsc.owly.math.state.StandardTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;

public class SimpleTrajectoryRegionQuery extends StandardTrajectoryRegionQuery {
  // TODO make static function for construction
  public SimpleTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    super(stateTimeRegion, new SparseStateTimeRegionMembers());
  }

  /** Region members, which were found in Region
   * for GUI as only 1 State is allowed in 1 Raster (for sparsity)
   * 
   * @return Collection<stateTime> the members of the sparse raster */
  public Collection<StateTime> getSparseDiscoveredMembers() {
    return ((SparseStateTimeRegionMembers) getStateTimeRegionCallback()).getMembers();
  }
}
