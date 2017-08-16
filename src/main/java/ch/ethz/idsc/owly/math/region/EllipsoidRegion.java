// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** EllipsoidRegion implements an axis aligned elliptic region in the vector space R^n.
 * 
 * The region also finds applications for other spaces, such as R^n x R^m
 * where axis depended scaling is desired. One use case is the Lotka-Volterra model.
 * 
 * Notice: evaluate(...) does not correspond to Euclidean distance */
public class EllipsoidRegion extends ImplicitFunctionRegion {
  private final Tensor center;
  private final Tensor invert;

  /** @param center of the ellipsoid
   * @param radius of the different axes with same number of entries as center
   * all components of radius must be non-negative.
   * if a component of radius is RealScalar.POSITIVE_INFINITY, this corresponds to a cylinder */
  public EllipsoidRegion(Tensor center, Tensor radius) {
    if (center.length() != radius.length())
      throw TensorRuntimeException.of(center, radius);
    if (!radius.equals(Ramp.of(radius))) // assert that radius are non-negative
      throw TensorRuntimeException.of(radius);
    // ---
    this.center = center;
    // FIXME JAN double 0.0 -> inf, symbolic 0 -> exception
    invert = radius.map(Scalar::reciprocal); // throws an exception if any radius == 0
  }

  @Override
  public Scalar evaluate(Tensor tensor) {
    return Norm._2SQUARED.of(tensor.subtract(center).pmul(invert)).subtract(RealScalar.ONE);
  }
}
