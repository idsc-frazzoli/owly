// code by jph and jl
package ch.ethz.idsc.owl.glc.core;

import ch.ethz.idsc.owl.data.tree.StateCostNode;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;

/** glc specific node
 * 
 * immutable except for children, parent, and depth which are only modified in
 * {@link GlcNode#addChild(GlcNode)} */
public interface GlcNode extends StateCostNode {
  /** creates {@link GlcNode} without parent and without descendants/children
   * 
   * @param flow used to reach this node
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

  /** @return flow between parent and this node. if this node is root, flow == null */
  Flow flow();

  /** @return coordinate in space-time of node */
  StateTime stateTime();

  /** @return cost from root plus min cost to goal */
  Scalar merit();

  /** Function gives the depth of the Node in respect to the first root.
   * The Depth is the number of ancestors until the root.
   * 
   * @return the depth of this node */
  int depth();

  /** function is only called by motion planners.
   * data structures that rely on the sorting by merit
   * may become invalid once the merit is set to a new value */
  void setMinCostToGoal(Scalar minCostToGoal);

  /** @return the difference in depth (numbers of ancestors till root) to the current root */
  int depthDifferenceToRoot();

  /** makes this Node root, by cutting connections to its parents */
  void makeRoot();
}
