// code by jph
package ch.ethz.idsc.owly.math.region;

import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

/** RegionUnion is a region that defines membership
 * to be member in either of a collection of {@link Region}s
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegionUnion.html">RegionUnion</a> */
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
