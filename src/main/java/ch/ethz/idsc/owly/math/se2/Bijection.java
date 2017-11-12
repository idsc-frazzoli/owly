package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Tensor;

public interface Bijection {
  TensorUnaryOperator forward();

  TensorUnaryOperator inverse();

  Tensor forward_se2();
}
