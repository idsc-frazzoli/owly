// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

/** the spherical region is a special case of an {@link EllipsoidRegion}.
 * 
 * {@link SphericalRegion} is implemented separately, because the implementation
 * 1) requires less operations than if treated as an elliptic case
 * 2) is numerically more stable in corner cases
 * 
 * the function returns the minimal Euclidean distance that is separating
 * the input coordinate from the spherical region
 * 
 * for radius == 0, the region reduces to a single point: the center */
public class SphericalRegion extends ImplicitFunctionRegion {
  private final Tensor center;
  private final Scalar radius;

  /** @param center
   * @param radius non-negative */
  public SphericalRegion(Tensor center, Scalar radius) {
    this.center = center;
    this.radius = radius;
    GlobalAssert.that(Scalars.lessEquals(RealScalar.ZERO, radius));
  }

  @Override
  public Scalar evaluate(Tensor x) {
    return Norm._2.of(x.subtract(center)).subtract(radius);
  }
}
