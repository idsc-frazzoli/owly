// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** evaluate does not correspond to Euclidean distance */
public class EllipsoidRegion extends ImplicitFunctionRegion {
  protected final Tensor center;
  private final Tensor invert;

  /** @param center of the Ellipsoid
   * @param radius of the different axes.
   * all components of radius must be non-negative.
   * if a component of radius is RealScalar.POSITIVE_INFINITY, this corresponds to a Cylinder */
  public EllipsoidRegion(Tensor center, Tensor radius) {
    this.center = center;
    if (!radius.equals(Ramp.of(radius))) // assert that radius are non-negative
      throw new RuntimeException();
    invert = radius.map(Scalar::invert); // throws an exception if any radius == 0
  }

  @Override
  public Scalar evaluate(Tensor tensor) {
    // FIXME needs math derivation
    Tensor delta = center.subtract(tensor).pmul(invert);
    return Norm._2.of(delta).subtract(RealScalar.ONE);
  }
}
