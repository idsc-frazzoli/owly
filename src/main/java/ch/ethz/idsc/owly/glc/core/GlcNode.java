// code by jl, jph
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
  @Override // from Node
  GlcNode parent();

  Flow flow();

  StateTime stateTime();

  /** @return cost from root plus min cost to goal */
  Scalar merit();

  // function is only called by motion planners.
  // data structures that rely on the sorting by merit
  // may become invalid once the merit is set to a new value
  void setMinCostToGoal(Scalar minCostToGoal);

  int depth();

  int reCalculateDepth();
}
