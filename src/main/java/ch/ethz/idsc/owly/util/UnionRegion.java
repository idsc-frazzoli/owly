// code by jph
package ch.ethz.idsc.owly.util;

import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

public class UnionRegion implements Region {
  final Collection<Region> collection;

  public UnionRegion(Collection<Region> collection) {
    this.collection = collection;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    boolean isMember = false;
    for (Region region : collection)
      isMember |= region.isMember(tensor);
    return isMember;
  }
}
