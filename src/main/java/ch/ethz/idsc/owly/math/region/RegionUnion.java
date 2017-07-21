// code by jph & jl
package ch.ethz.idsc.owly.math.region;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

/** RegionUnion is a region that defines membership
 * to be member in either of a collection of {@link Region}s
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegionUnion.html">RegionUnion</a> */
public class RegionUnion implements Region {
  /** combines a collection of {@link Region}s into one Region.
   * Membership is defined as membership in any of the regions in the collection.
   * The input collection is not copied but used by reference.
   * Modification to outside collection have effect on this region.
   * 
   * The function name is inspired by {@link ByteBuffer#wrap(byte[])}.
   * 
   * @param collection collection of Regions
   * @return the combined Regions */
  public static Region wrap(Collection<Region> collection) {
    return new RegionUnion(collection);
  }

  /** @param regions Regions to be combined
   * @return Region, consisting from the combined inputed Regions */
  public static Region of(Region... regions) {
    return new RegionUnion(Arrays.asList(regions));
  }

  // ---
  private final Collection<Region> collection;

  public RegionUnion(Collection<Region> collection) { // Constructor made public
    this.collection = collection;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    boolean isMember = false;
    for (Region region : collection)
      isMember |= region.isMember(tensor);
    return isMember;
    // TODO try this alternative on a working/test case: works with RegionUnionTest
    // return collection.stream().parallel() //
    // .filter(region -> region.isMember(tensor)) //
    // .findAny().isPresent();
  }
}
