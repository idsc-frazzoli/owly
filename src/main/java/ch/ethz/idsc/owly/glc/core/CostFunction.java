// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface CostFunction {
  Scalar cost(Trajectory trajectory, Tensor u);

  double getLipschitz();
}
