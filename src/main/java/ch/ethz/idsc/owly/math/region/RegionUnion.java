// code by jph & jl
package ch.ethz.idsc.owly.math.region;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.tensor.Tensor;

/** RegionUnion is a region that defines membership
 * to be member in either of a collection of {@link TensorRegion}s
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegionUnion.html">RegionUnion</a> */
public class RegionUnion implements TensorRegion {
  /** combines a collection of {@link TensorRegion}s into one Region.
   * Membership is defined as membership in any of the regions in the collection.
   * The input collection is not copied but used by reference.
   * Modification to outside collection have effect on this region.
   * 
   * The function name is inspired by {@link ByteBuffer#wrap(byte[])}.
   * 
   * @param collection collection of Regions
   * @return the combined Regions */
  public static TensorRegion wrap(Collection<TensorRegion> collection) {
    return new RegionUnion(collection);
  }

  /** @param regions to union
   * @return the union of the given regions */
  public static TensorRegion of(TensorRegion... regions) {
    return new RegionUnion(Arrays.asList(regions));
  }

  // ---
  private final Collection<TensorRegion> collection;

  private RegionUnion(Collection<TensorRegion> collection) {
    this.collection = collection;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return collection.stream().parallel().anyMatch(region -> region.isMember(tensor));
  }
}
