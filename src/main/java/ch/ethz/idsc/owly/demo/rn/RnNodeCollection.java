// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.Collection;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.data.nd.NdCenterInterface;
import ch.ethz.idsc.owly.data.nd.NdCluster;
import ch.ethz.idsc.owly.data.nd.NdEntry;
import ch.ethz.idsc.owly.data.nd.NdMap;
import ch.ethz.idsc.owly.data.nd.NdTreeMap;
import ch.ethz.idsc.owly.rrts.core.RrtsNode;
import ch.ethz.idsc.owly.rrts.core.RrtsNodeCollection;
import ch.ethz.idsc.tensor.Tensor;

/** collection of nodes in R^n backed by a n-dimensional uniform tree
 * in 2-d, the data structure is a quad tree
 * in 3-d, the data structure is a octree */
public class RnNodeCollection implements RrtsNodeCollection {
  private final NdMap<RrtsNode> ndMap;

  public RnNodeCollection(Tensor lbounds, Tensor ubounds) {
    ndMap = new NdTreeMap<>(lbounds, ubounds, 5, 20);
  }

  @Override
  public void insert(RrtsNode rrtsNode) {
    ndMap.add(rrtsNode.state(), rrtsNode);
  }

  @Override
  public int size() {
    return ndMap.size();
  }

  @Override
  public Collection<RrtsNode> nearTo(Tensor end, int k_nearest) {
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(end);
    NdCluster<RrtsNode> cluster = ndMap.buildCluster(distanceInterface, k_nearest);
    // System.out.println("considered " + cluster.considered() + " " + ndMap.size());
    return cluster.stream().map(NdEntry::value).collect(Collectors.toList());
  }

  @Override
  public Collection<RrtsNode> nearFrom(Tensor start, int k_nearest) {
    return nearTo(start, k_nearest);
  }
}
