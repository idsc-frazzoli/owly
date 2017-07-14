// code by jph & jl
package ch.ethz.idsc.owly.math.region;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

import ch.ethz.idsc.tensor.Tensor;

/** RegionUnion is a region that defines membership
 * to be member in either of a collection of {@link Region}s
 * 
 * <p>inspired by
 * <a href="https://reference.wolfram.com/language/ref/RegionUnion.html">RegionUnion</a> */
public class RegionUnion implements Region {
  /** @param regions Regions to be combined
   * @return Region, consisting from the combined inputed Regions */
  public static Region of(Region... regions) {
    return new RegionUnion(Arrays.asList(regions));
  }

  /** Combines a List of Regions into 1 Region
   * 
   * @param collection collection of Regions
   * @return the combined Regions */
  public static Region of(Collection<Region> collection) {
    return new RegionUnion(collection);
  }

  // ---
  private final Collection<Region> collection;

  public RegionUnion(Collection<Region> collection) { // Constructor made public
    this.collection = collection;
  }

  // TODO functions not used yet
  public final Region add(Region region) {
    return add(Collections.singleton(region));
  }

  public final Region add(Collection<Region> collectionToAdd) {
    collection.addAll(collectionToAdd);
    return this;
  }

  public final Region removeAll(Collection<Region> collectionToRemove) {
    collection.removeAll(collectionToRemove);
    return this;
  }

  public final Region remove(Region region) {
    return removeAll(Collections.singleton(region));
  }

  @Override
  public boolean isMember(Tensor tensor) {
    boolean isMember = false;
    for (Region region : collection)
      isMember |= region.isMember(tensor);
    return isMember;
    // TODO try this alternative on a working case
    // return collection.stream().parallel() //
    // .filter(region -> region.isMember(tensor)) //
    // .findAny().isPresent();
  }
}
