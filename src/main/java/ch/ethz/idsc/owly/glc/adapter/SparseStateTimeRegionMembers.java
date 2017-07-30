// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Collection;
import java.util.Collections;

import ch.ethz.idsc.owly.data.LinearRasterMap;
import ch.ethz.idsc.owly.data.RasterMap;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeRegionCallback;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class SparseStateTimeRegionMembers implements StateTimeRegionCallback {
  /** magic constants of scale are not universal but are suitable for most examples */
  private final RasterMap<StateTime> rasterMap = new LinearRasterMap<>(Tensors.vector(10, 10));

  @Override
  public void notify_isMember(StateTime stateTime) {
    Tensor x = stateTime.state();
    if (1 < x.length())
      rasterMap.put(x.extract(0, 2), stateTime);
  }

  public Collection<StateTime> getMembers() {
    return Collections.unmodifiableCollection(rasterMap.values());
  }
}
