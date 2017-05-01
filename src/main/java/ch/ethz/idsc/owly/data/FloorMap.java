// code by jph
package ch.ethz.idsc.owly.data;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Floor;

public class FloorMap<T> {
  private final Tensor scale;
  private final Map<Tensor, T> map = new HashMap<>();

  public FloorMap(Tensor scale) {
    this.scale = scale;
  }

  public void put(Tensor tensor, T value) {
    map.put(Floor.of(tensor.extract(0, scale.length()).pmul(scale)), value);
  }

  public Collection<T> values() {
    return map.values();
  }
}
