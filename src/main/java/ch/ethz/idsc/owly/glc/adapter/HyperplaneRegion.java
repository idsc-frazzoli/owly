// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.ImplicitFunctionRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class HyperplaneRegion extends ImplicitFunctionRegion {
  final Tensor normal;
  final Scalar threshold;

  public HyperplaneRegion(Tensor normal, Scalar threshold) {
    this.normal = normal;
    this.threshold = threshold;
  }

  @Override
  public Scalar apply(Tensor x) {
    return (Scalar) x.dot(normal).subtract(threshold);
  }
}
