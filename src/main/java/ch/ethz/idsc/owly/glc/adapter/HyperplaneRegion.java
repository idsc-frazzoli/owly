// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.ImplicitFunctionRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

// TODO make a test and document!!!
public class HyperplaneRegion extends ImplicitFunctionRegion {
  final Tensor normal;
  final Scalar threshold;

  /** normal is not required to have Euclidean length 1
   * 
   * @param normal is orthogonal to hyperplane
   * @param threshold */
  public HyperplaneRegion(Tensor normal, Scalar threshold) {
    this.normal = normal;
    this.threshold = threshold;
  }

  @Override
  public Scalar evaluate(Tensor x) {
    return (Scalar) x.dot(normal).subtract(threshold);
  }
}
