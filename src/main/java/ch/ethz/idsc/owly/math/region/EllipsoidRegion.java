// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** evaluate does not correspond to Euclidean distance */
public class EllipsoidRegion extends ImplicitFunctionRegion {
  private final Tensor center;
  private final Tensor invert;

  /** @param center of the Ellipsoid
   * @param radius of the different axes.
   * all components of radius must be non-negative.
   * if a component of radius is RealScalar.POSITIVE_INFINITY, this corresponds to a Cylinder */
  public EllipsoidRegion(Tensor center, Tensor radius) {
    this.center = center;
    if (!radius.equals(Ramp.of(radius))) // assert that radius are non-negative
      throw TensorRuntimeException.of(radius);
    invert = radius.map(Scalar::reciprocal); // throws an exception if any radius == 0
  }

  @Override
  public Scalar evaluate(Tensor tensor) {
    return Norm._2SQUARED.of(tensor.subtract(center).pmul(invert)).subtract(RealScalar.ONE);
  }
}
