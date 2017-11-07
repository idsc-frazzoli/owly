// code by jph
package ch.ethz.idsc.owly.math.region;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

/** RegionIntersection is a region that defines membership
 * to be member in all of a collection of {@link TensorRegion}s
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegionIntersection.html">RegionIntersection</a> */
public class RegionIntersection implements TensorRegion {
  /** combines a collection of {@link TensorRegion}s into one Region.
   * Membership is defined as membership in all of the regions in the collection.
   * The input collection is not copied but used by reference.
   * Modification to outside collection have effect on this region.
   * 
   * The function name is inspired by {@link ByteBuffer#wrap(byte[])}.
   * 
   * @param collection collection of Regions
   * @return the intersection of the given regions */
  public static TensorRegion wrap(Collection<TensorRegion> collection) {
    return new RegionIntersection(collection);
  }

  /** @param regions to be combined
   * @return the intersection of the given regions */
  public static TensorRegion of(TensorRegion... regions) {
    return new RegionIntersection(Arrays.asList(regions));
  }

  // ---
  private final Collection<TensorRegion> collection;

  private RegionIntersection(Collection<TensorRegion> collection) {
    this.collection = collection;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return collection.stream().allMatch(region -> region.isMember(tensor));
  }
}
