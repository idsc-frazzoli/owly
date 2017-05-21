// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

/** evaluate does not correspond to Euclidean distance */
public class EllipsoidRegion extends ImplicitFunctionRegion {
  protected final Tensor center;
  private final Tensor invert;

  public EllipsoidRegion(Tensor center, Tensor radius) {
    this.center = center;
    invert = radius.map(Scalar::invert);
  }

  @Override
  public Scalar evaluate(Tensor tensor) {
    // FIXME needs math derivation
    Tensor delta = center.subtract(tensor).pmul(invert);
    return Norm._2.of(delta).subtract(RealScalar.ONE);
  }
}
