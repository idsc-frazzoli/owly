// code by jph & jl
package ch.ethz.idsc.owly.demo.deltaxt;

import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Join;

public class DeltaxtStateSpaceModel implements StateSpaceModel {
  private static final Tensor AFFINE_ONE = Tensors.vector(1);
  private final ImageGradient imageGradient;
  private final Scalar maxInput;

  public DeltaxtStateSpaceModel(ImageGradient imageGradient, Scalar maxInput) {
    this.imageGradient = imageGradient;
    this.maxInput = maxInput;
  }

  @Override
  public Tensor f(Tensor x, Tensor u) {
    int toIndex = x.length() - 1;
    if (toIndex != 2)
      throw TensorRuntimeException.of(x, u);
    Tensor fxy = imageGradient.rotate(x.extract(0, toIndex));
    return Join.of(fxy.add(u), AFFINE_ONE);
  }

  @Override
  public Scalar getLipschitz() {
    // maxNorm is very big--> therefore eta with R^(1+LF) is huge? real lipschitz?
    Scalar n = RealScalar.of(4); // dimensions of StateSpace + Dimensions of InputSpace
    // lipschitz constant on vector-valued function from:
    // https://math.stackexchange.com/questions/1132078/proof-that-a-vector-valued-function-is-lipschitz-continuous-on-a-closed-rectangl
    return imageGradient.maxNormGradient().add(maxInput).multiply(n);
  }

  public Scalar getMaxInput() {
    return maxInput;
  }

  public Scalar getMaxPossibleChange() {
    return maxInput.add(imageGradient.maxNormGradient());
    // TODO modify due to time state
  }
}
