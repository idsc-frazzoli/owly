// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.util.function.UnaryOperator;

import ch.ethz.idsc.owly.math.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** suitable for time-variant state space models */
public enum VectorFields {
  ;
  public static Tensor of(StateSpaceModel stateSpaceModel, Tensor points, Tensor fallback_u, Scalar factor) {
    UnaryOperator<Tensor> unaryOperator = //
        x -> Tensors.of(x, x.add(stateSpaceModel.f(x, fallback_u).multiply(factor)));
    return Tensors.vector(index -> unaryOperator.apply(points.get(index)), points.length());
  }
}
