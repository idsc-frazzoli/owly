// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class NdListMap<V> implements NdMap<V> {
  List<NdEntry<V>> list = new ArrayList<>();

  @Override
  public void add(Tensor location, V value) {
    list.add(new NdEntry<>(location, value));
  }

  @Override
  public int size() {
    return list.size();
  }

  @Override
  public NdCluster<V> buildCluster(Tensor center, int size, NdDistanceInterface distancer) {
    return new NdCluster<>(list.stream().sorted((e1, e2) -> Scalars.compare( //
        distancer.apply(e1.location, center), //
        distancer.apply(e2.location, center))) //
        .limit(size));
  }
}
