// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.List;

import ch.ethz.idsc.owly.math.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface CostFunction {
  /** @param trajectory
   * @param flow
   * @return cost of trajectory along flow */
  Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow);

  /** if a lower bound of the cost to goal are unknown,
   * the function should return ZeroScalar.get()
   * 
   * it is imperative that the function does not return a
   * greater number than is absolutely necessary to reach the goal
   * TODO comment why
   * 
   * @param x
   * @return lower bound of cost to goal */
  Scalar minCostToGoal(Tensor x);
}
