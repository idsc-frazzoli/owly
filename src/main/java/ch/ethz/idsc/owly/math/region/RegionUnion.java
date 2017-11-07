// code by jph & jl
package ch.ethz.idsc.owly.math.region;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collection;

/** RegionUnion is a region that defines membership
 * to be member in either of a collection of {@link TensorRegion}s
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegionUnion.html">RegionUnion</a> */
public class RegionUnion<T> implements Region<T> {
  /** combines a collection of {@link TensorRegion}s into one Region.
   * Membership is defined as membership in any of the regions in the collection.
   * The input collection is not copied but used by reference.
   * Modification to outside collection have effect on this region.
   * 
   * The function name is inspired by {@link ByteBuffer#wrap(byte[])}.
   * 
   * @param collection collection of Regions
   * @return the combined Regions */
  // TODO function is misnomer
  public static <T> Region<T> wrap(Collection<Region<T>> collection) {
    return new RegionUnion<T>(collection);
  }

  /** @param regions to union
   * @return the union of the given regions */
  @SuppressWarnings("unchecked")
  public static <T> Region<T> of(Region<T>... regions) {
    return new RegionUnion<T>(Arrays.asList(regions));
  }

  // ---
  private final Collection<Region<T>> collection;

  private RegionUnion(Collection<Region<T>> collection) {
    this.collection = collection;
  }

  @Override
  public boolean isMember(T type) {
    return collection.stream().parallel().anyMatch(region -> region.isMember(type));
  }
}
