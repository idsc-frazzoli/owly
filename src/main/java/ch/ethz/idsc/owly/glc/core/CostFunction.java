// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.util.Flow;
import ch.ethz.idsc.tensor.Scalar;

public interface CostFunction {
  Scalar cost(Trajectory trajectory, Flow u);

  Scalar getLipschitz();
}
