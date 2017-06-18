// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import java.util.Collection;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.nd.NdCluster;
import ch.ethz.idsc.owly.data.nd.NdDistanceInterface;
import ch.ethz.idsc.owly.data.nd.NdTreeMap;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.tensor.Tensor;

/** collection of nodes in R^n backed by a n-dimensional uniform tree
 * in 2-d, the data structure is a quad tree
 * in 3-d, the data structure is a octree */
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
    NdCluster<RrtsNode> cluster = ndTreeMap.buildCluster(end, k_nearest, NdDistanceInterface.EUCLIDEAN_SQUARED);
    return cluster.stream().map(p -> p.value).collect(Collectors.toList());
  }

  @Override
  public Collection<RrtsNode> nearFrom(Tensor start, int k_nearest) {
    return nearTo(start, k_nearest);
  }
}
