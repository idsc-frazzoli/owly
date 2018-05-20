// code by jph
package ch.ethz.idsc.owl.math.region;

import java.nio.ByteBuffer;
import java.util.Collection;

/** RegionIntersection is a region that defines membership
 * to be member in all of a collection of {@link Region}s
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegionIntersection.html">RegionIntersection</a> */
public class RegionIntersection<T> implements Region<T> {
  /** combines a collection of {@link Region}s into one Region.
   * Membership is defined as membership in all of the regions in the collection.
   * The input collection is not copied but used by reference.
   * Modification to outside collection have effect on this region.
   * 
   * The function name is inspired by {@link ByteBuffer#wrap(byte[])}.
   * 
   * @param collection collection of Regions
   * @return the intersection of the given regions */
  public static <T> Region<T> wrap(Collection<Region<T>> collection) {
    return new RegionIntersection<>(collection);
  }

  // ---
  private final Collection<Region<T>> collection;

  private RegionIntersection(Collection<Region<T>> collection) {
    this.collection = collection;
  }

  @Override // from Region
  public boolean isMember(T tensor) {
    return collection.stream().allMatch(region -> region.isMember(tensor));
  }
}
