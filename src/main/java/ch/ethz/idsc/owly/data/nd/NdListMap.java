// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.tensor.Tensor;

public class NdListMap<V> implements NdMap<V> {
  List<NdPair<V>> list = new ArrayList<>();

  @Override
  public void add(Tensor location, V value) {
    list.add(new NdPair<>(location, value));
  }

  @Override
  public int size() {
    return list.size();
  }

  @Override
  public NdCluster<V> buildCluster(NdCenterInterface ndCenter, int limit) {
    return new NdCluster<>(list, ndCenter, limit);
  }
}
