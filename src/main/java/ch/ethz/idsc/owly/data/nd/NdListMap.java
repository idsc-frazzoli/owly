// code by jph
package ch.ethz.idsc.owly.data.nd;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

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
  public NdCluster<V> buildCluster(Tensor center, int size, NdDistanceInterface distancer) {
    List<NdEntry<V>> entries = list.stream() //
        .map(pair -> new NdEntry<>(pair, distancer.apply(pair.location, center))) //
        .collect(Collectors.toList());
    return new NdCluster<>(center, entries.stream().sorted(NdEntryComparators.INCREASING).limit(size));
  }
}
