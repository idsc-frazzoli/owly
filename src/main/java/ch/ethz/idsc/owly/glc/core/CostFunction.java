// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.Scalar;

public interface CostFunction {
  /** @param trajectory
   * @param u
   * @return cost of trajectory along flow u */
  Scalar cost(List<StateTime> trajectory, Flow u);
}
