// code by jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;

public enum GlcNodes {
  ;
  // ---
  /** @param flow
   * @param stateTime
   * @param costFromRoot
   * @param minCostToGoal
   * @return */
  public static GlcNode of(Flow flow, StateTime stateTime, Scalar costFromRoot, Scalar minCostToGoal) {
    return new GlcNodeImpl(flow, stateTime, costFromRoot, minCostToGoal);
  }
}
