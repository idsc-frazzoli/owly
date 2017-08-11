// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

//TODO JAN: implementation here good? or somewhere else better?
public class EuclideanDistanceDiscoverRegion implements Region {
  final Region region;
  final Scalar distance;
  final Tensor origin;

  public EuclideanDistanceDiscoverRegion(Region region, Tensor Origin, Scalar distance) {
    this.region = region;
    origin = Origin;
    this.distance = distance;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    if (Scalars.lessEquals(Norm._2.of(tensor.subtract(origin)), distance))
      return region.isMember(tensor);
    return false;
  }
}
