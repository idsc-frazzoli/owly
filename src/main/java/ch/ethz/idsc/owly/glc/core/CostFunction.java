// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;

public interface CostFunction extends HeuristicFunction {
  /** @param from
   * @param trajectory
   * @param flow
   * @return cost of trajectory along flow */
  Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow);
}
