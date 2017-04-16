// code by jph
package ch.ethz.idsc.owly.util;

import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

public class UnionRegion implements Region {
  private final Collection<Region> collection;

  public static Region of(Region... regions) {
    return new UnionRegion(Arrays.asList(regions));
  }

  private UnionRegion(Collection<Region> collection) {
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
