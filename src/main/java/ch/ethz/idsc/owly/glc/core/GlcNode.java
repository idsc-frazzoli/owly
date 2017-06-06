// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;

/** glc specific node
 * 
 * immutable except for children, parent, and depth which are only modified in
 * {@link GlcNode#addChild(GlcNode)} */
public interface GlcNode extends StateCostNode {
  /** @param flow
   * @param stateTime
   * @param costFromRoot
   * @param minCostToGoal
   * @return */
  static GlcNode of(Flow flow, StateTime stateTime, Scalar costFromRoot, Scalar minCostToGoal) {
    return new GlcNodeImpl(flow, stateTime, costFromRoot, minCostToGoal);
  }

  /***************************************************/
  @Override // from Node
  GlcNode parent();

  Flow flow();

  StateTime stateTime();

  /** @return cost from root plus min cost to goal */
  Scalar merit();

  int depth();

  // function is only called by motion planners.
  // data structures that rely on the sorting by merit
  // may become invalid once the merit is set to a new value
  // during development, function is public, but later it would be nice to hide this function
  void setMinCostToGoal(Scalar minCostToGoal);

  // during development, function is public, but later it would be nice to hide this function
  int reCalculateDepth();
}
