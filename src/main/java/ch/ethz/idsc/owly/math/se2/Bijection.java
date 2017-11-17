package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;

public interface Bijection {
  TensorUnaryOperator forward();

  TensorUnaryOperator inverse();

  Tensor forward_se2();
}
