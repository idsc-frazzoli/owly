// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.ImplicitFunctionRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Normalize;

public class HyperplaneRegion extends ImplicitFunctionRegion {
  private final Tensor normal; // orthogonal to hyperplane pointing outside
  private final Scalar distanceFromZero;

  /** orthogonal is not required to have Euclidean length 1
   * but will be normalized inside the constructor
   * 
   * distanceFromZero is the amount needed to reach the region
   * starting from position (0,...,0).
   * That means, if distanceFromZero is negative, (0,...,0) is inside the region
   * 
   * @param orthogonal is orthogonal to hyperplane
   * @param distanceFromZero */
  public HyperplaneRegion(Tensor orthogonal, Scalar distanceFromZero) {
    this.normal = Normalize.of(orthogonal);
    this.distanceFromZero = distanceFromZero;
  }

  @Override
  public Scalar evaluate(Tensor x) {
    return (Scalar) x.dot(normal).add(distanceFromZero);
  }
}
