// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.red.Norm2Squared;

/** EllipsoidRegion implements an axis aligned elliptic region in the vector space R^n.
 * 
 * The region also finds applications for other spaces, such as R^n x R^m
 * where axis depended scaling is desired. One use case is the Lotka-Volterra model.
 * 
 * Notice: evaluate(...) does not correspond to Euclidean distance
 * 
 * @see SphericalRegion */
public class EllipsoidRegion extends ImplicitFunctionRegion {
  private final Tensor center;
  private final Tensor invert;

  /** The components of radius have to be strictly positive.
   * For a radius equals to zero we would run into numerical trouble:
   * distance Inf when outside, and NaN when at the center
   * 
   * @param center of the ellipsoid
   * @param radius of the different axes with same number of entries as center
   * all components of radius must be strictly positive.
   * if a component of radius is DoubleScalar.POSITIVE_INFINITY, this corresponds to a cylinder
   * @see SphericalRegion */
  public EllipsoidRegion(Tensor center, Tensor radius) {
    if (center.length() != radius.length())
      throw TensorRuntimeException.of(center, radius);
    if (radius.stream() // <- assert that radius are strictly positive
        .map(Scalar.class::cast) //
        .anyMatch(scalar -> Scalars.lessEquals(scalar, RealScalar.ZERO)))
      throw TensorRuntimeException.of(radius);
    // ---
    this.center = center;
    invert = radius.map(Scalar::reciprocal);
  }

  @Override
  public Scalar evaluate(Tensor tensor) {
    return Norm2Squared.ofVector(tensor.subtract(center).pmul(invert)).subtract(RealScalar.ONE);
  }
}
