// code by jph and jl
package ch.ethz.idsc.owly.demo.delta;

import ch.ethz.idsc.owl.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** an upper bound of the speed of an entity in the river delta is
 * imageGradient.maxNormGradient() + |u_max| */
public class DeltaStateSpaceModel implements StateSpaceModel {
  private final ImageGradient imageGradient;

  /** @param imageGradient
   * @param maxInput positive */
  public DeltaStateSpaceModel(ImageGradient imageGradient) {
    this.imageGradient = imageGradient;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    return imageGradient.rotate(x).add(u);
  }

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 | */
  @Override
  public Scalar getLipschitz() {
    return imageGradient.maxNormGradient(); // DO NOT CHANGE
  }
}
