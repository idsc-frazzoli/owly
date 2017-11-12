// code by jph and jl
package ch.ethz.idsc.owly.demo.delta;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Sign;

/** TODO see below:
 * this example shows that the lipschitz constant depends on the f(x,u)
 * as well as the set of which the u's are drawn from.
 * that means, the lipschitz function should not be part of StateSpaceModel
 * but a new object that combines these 2 concepts, i.e. a collection of flows */
public class DeltaStateSpaceModel implements StateSpaceModel {
  private final ImageGradient imageGradient;
  private final Scalar maxInput;

  /** @param imageGradient
   * @param maxInput positive */
  public DeltaStateSpaceModel(ImageGradient imageGradient, Scalar maxInput) {
    GlobalAssert.that(Sign.isPositive(maxInput));
    this.imageGradient = imageGradient;
    this.maxInput = maxInput;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    return imageGradient.rotate(x).add(u);
  }

  @Override
  public Scalar getLipschitz() {
    return imageGradient.maxNormGradient().add(maxInput);
  }
}
