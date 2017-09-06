// code by jph and jl
package ch.ethz.idsc.owly.demo.delta;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class DeltaStateSpaceModel implements StateSpaceModel {
  private final ImageGradient imageGradient;
  private final Scalar maxInput;

  /** @param imageGradient
   * @param maxInput may be null in case lipschitz computation is not needed */
  public DeltaStateSpaceModel(ImageGradient imageGradient, Scalar maxInput) {
    this.imageGradient = imageGradient;
    this.maxInput = maxInput;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    return imageGradient.rotate(x).add(u);
  }

  @Override
  public Scalar getLipschitz() {
    // maxNorm is very big--> therefore eta with R^(1+LF) is huge? real lipschitz?
    Scalar n = RealScalar.of(4); // dimensions of StateSpace + Dimensions of InputSpace
    // lipschitz constant on vector-valued function from:
    // https://math.stackexchange.com/questions/1132078/proof-that-a-vector-valued-function-is-lipschitz-continuous-on-a-closed-rectangl
    return imageGradient.maxNormGradient().add(maxInput).multiply(n);
  }

  public Scalar getMaxPossibleChange() {
    return maxInput.add(imageGradient.maxNormGradient());
  }
}
