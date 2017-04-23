// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.ImplicitFunctionRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public class EllipsoidRegion extends ImplicitFunctionRegion {
  protected final Tensor center;
  private final Tensor invert;

  public EllipsoidRegion(Tensor center, Tensor radius) {
    this.center = center;
    invert = radius.map(Scalar::invert);
  }

  @Override
  public Scalar apply(Tensor tensor) {
    // TODO perhaps this function should return the real euclidean distance in case ellipsoid is a sphere!!!
    // FIXME needs math derivation
    Tensor delta = center.subtract(tensor).pmul(invert);
    return Norm._2.of(delta).subtract(RealScalar.ONE);
  }
}
