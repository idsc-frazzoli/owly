// code by jl
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.region.RegionIntersection;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public enum EuclideanDistanceDiscoverRegion {
  ;
  /** @param region
   * @param origin
   * @param distance
   * @return */
  public static Region of(Region region, Tensor origin, Scalar distance) {
    if (origin.length() == 3)
      // TODO Enhance to any statespace in the assumption that the first 2 are x and y
      return RegionIntersection.of(new EllipsoidRegion(origin, Tensors.of(distance, distance, DoubleScalar.POSITIVE_INFINITY)), region);
    return RegionIntersection.of(new SphericalRegion(origin, distance), region);
  }
}
