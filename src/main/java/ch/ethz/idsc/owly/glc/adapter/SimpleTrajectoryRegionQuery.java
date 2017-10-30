// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StandardTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeCollector;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;

public class SimpleTrajectoryRegionQuery extends StandardTrajectoryRegionQuery implements StateTimeCollector {
  /** @param region
   * @return */
  public static TrajectoryRegionQuery timeInvariant(Region region) {
    return new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(region));
  }

  /** @param stateTimeRegion */
  public SimpleTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    super(stateTimeRegion, new SparseStateTimeRegionMembers());
  }

  /** Region members, which were found in Region
   * for GUI as only 1 State is allowed in 1 Raster (for sparsity)
   * 
   * @return Collection<stateTime> the members of the sparse raster */
  @Override
  public Collection<StateTime> getMembers() {
    return ((SparseStateTimeRegionMembers) getStateTimeRegionCallback()).getMembers();
  }
}
