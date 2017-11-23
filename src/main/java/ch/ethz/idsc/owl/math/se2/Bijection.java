// code by jph
package ch.ethz.idsc.owl.math.se2;

import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;

public interface Bijection {
  TensorUnaryOperator forward();

  TensorUnaryOperator inverse();
}
