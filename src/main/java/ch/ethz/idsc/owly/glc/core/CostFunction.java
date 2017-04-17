// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.Scalar;

public interface CostFunction {
  /** @param trajectory
   * @param u
   * @return cost of trajectory along flow u */
  Scalar cost(Trajectory trajectory, Flow u);

  Scalar getLipschitz();
}
