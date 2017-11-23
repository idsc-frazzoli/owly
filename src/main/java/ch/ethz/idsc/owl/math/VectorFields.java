// code by jph
package ch.ethz.idsc.owl.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;

/** suitable for time-variant state space models */
public enum VectorFields {
  ;
  public static Tensor of(StateSpaceModel stateSpaceModel, Tensor points, Tensor fallback_u, Scalar factor) {
    TensorUnaryOperator tensorUnaryOperator = //
        x -> Tensors.of(x, x.add(stateSpaceModel.f(x, fallback_u).multiply(factor)));
    return Tensors.vector(index -> tensorUnaryOperator.apply(points.get(index)), points.length());
  }
}
