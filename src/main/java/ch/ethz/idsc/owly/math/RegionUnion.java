// code by jph
package ch.ethz.idsc.owly.math;

import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

/** TODO ref mathematica
 * RegionUnion */
public class RegionUnion implements Region {
  private final Collection<Region> collection;

  public static Region of(Region... regions) {
    return new RegionUnion(Arrays.asList(regions));
  }

  private RegionUnion(Collection<Region> collection) {
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
