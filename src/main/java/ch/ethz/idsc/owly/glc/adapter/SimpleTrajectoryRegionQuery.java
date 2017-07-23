// code by bapaden and jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.data.LinearRasterMap;
import ch.ethz.idsc.owly.data.RasterMap;
import ch.ethz.idsc.owly.math.state.AbstractTrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.Serialization;

public class SimpleTrajectoryRegionQuery extends AbstractTrajectoryRegionQuery {
  private final StateTimeRegion stateTimeRegion;
  // TODO magic constants of scale are not universal
  private RasterMap<StateTime> discoveredSparseMembers = new LinearRasterMap<>(Tensors.vector(10, 10));
  // TODO make final again?

  public SimpleTrajectoryRegionQuery(StateTimeRegion stateTimeRegion) {
    this.stateTimeRegion = stateTimeRegion;
  }

  public SimpleTrajectoryRegionQuery(SimpleTrajectoryRegionQuery simpleTrajectoryRegionQuery) {
    try {
      discoveredSparseMembers = Serialization.copy(simpleTrajectoryRegionQuery.discoveredSparseMembers);
    } catch (Exception e) {
      e.printStackTrace();
    }
    this.stateTimeRegion = simpleTrajectoryRegionQuery.stateTimeRegion;
  }

  @Override
  public final int firstMember(List<StateTime> trajectory) {
    int index = -1;
    for (StateTime stateTime : trajectory) {
      ++index;
      if (stateTimeRegion.isMember(stateTime)) {
        Tensor x = stateTime.x();
        if (1 < x.length())
          discoveredSparseMembers.put(x.extract(0, 2), stateTime);
        return index;
      }
    }
    return NOMATCH;
  }

  /** Region members, which were found in Region
   * for GUI as only 1 State is allowed in 1 Raster (for sparsity)
   * 
   * @return Collection<stateTime> the members of the sparse Raster */
  public Collection<StateTime> getSparseDiscoveredMembers() {
    return discoveredSparseMembers.values();
  }
}
