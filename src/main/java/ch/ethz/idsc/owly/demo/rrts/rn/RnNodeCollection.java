// code by jph
package ch.ethz.idsc.owly.demo.rrts.rn;

import java.util.Collection;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.cluster.Cluster;
import ch.ethz.idsc.owly.data.cluster.DistanceInterface;
import ch.ethz.idsc.owly.data.cluster.NdTreeMap;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.tensor.Tensor;

class RnNodeCollection implements RrtsNodeCollection {
  private final NdTreeMap<RrtsNode> ndTreeMap;

  public RnNodeCollection(Tensor lbounds, Tensor ubounds) {
    ndTreeMap = new NdTreeMap<>(lbounds, ubounds, 40, 12); // TODO magic const
  }

  @Override
  public void insert(RrtsNode rrtsNode) {
    ndTreeMap.add(rrtsNode.state(), rrtsNode);
  }

  @Override
  public int size() {
    return ndTreeMap.size();
  }

  @Override
  public Collection<RrtsNode> nearTo(Tensor end, int k_nearest) {
    Cluster<RrtsNode> cluster = ndTreeMap.buildCluster(end, k_nearest, DistanceInterface.EUCLIDEAN_SQUARED);
    return cluster.stream().map(p -> p.value).collect(Collectors.toList());
  }

  @Override
  public Collection<RrtsNode> nearFrom(Tensor start, int k_nearest) {
    return nearTo(start, k_nearest);
  }
}
