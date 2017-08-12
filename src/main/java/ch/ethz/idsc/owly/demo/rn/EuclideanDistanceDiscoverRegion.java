// code by jl
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionIntersection;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public enum EuclideanDistanceDiscoverRegion {
  ;
  /** @param region
   * @param origin
   * @param distance
   * @return */
  public static Region of(Region region, Tensor origin, Scalar distance) {
    return RegionIntersection.of(new SphericalRegion(origin, distance), region);
  }
}
