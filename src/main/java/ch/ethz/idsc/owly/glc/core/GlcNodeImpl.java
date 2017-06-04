// code by bapaden, jl and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.owly.data.tree.AbstractNode;
import ch.ethz.idsc.owly.data.tree.Nodes;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** glc specific node
 * 
 * immutable except for children, parent, and depth which are only modified in
 * {@link GlcNodeImpl#addChild(GlcNodeImpl)} */
/* package */ class GlcNodeImpl extends AbstractNode<GlcNodeImpl> implements GlcNode {
  private final Map<Flow, GlcNodeImpl> children = new HashMap<>();
  /** flow is null for root node */
  private final Flow flow;
  private final StateTime stateTime;
  private final Scalar costFromRoot;
  private Scalar merit;
  /** depth == 0 for root node, otherwise depth > 0 */
  private int depth = 0;

  /** @param flow that got us to this Node from the parent, or null when this Node is the root
   * @param stateTime
   * @param costFromRoot
   * @param minCostToGoal */
  GlcNodeImpl(Flow flow, StateTime stateTime, Scalar costFromRoot, Scalar minCostToGoal) {
    this.flow = flow;
    this.stateTime = stateTime;
    this.costFromRoot = costFromRoot;
    setMinCostToGoal(minCostToGoal);
  }

  @Override // from Node
  public Collection<GlcNodeImpl> children() {
    return Collections.unmodifiableCollection(children.values());
  }

  @Override // from StateCostNode
  public Tensor state() {
    return stateTime.x();
  }

  @Override // from StateCostNode
  public Scalar costFromRoot() {
    return costFromRoot;
  }

  @Override // from AbstractNode
  protected boolean protected_insertChild(GlcNodeImpl child) {
    boolean inserted = !children.containsKey(child.flow);
    child.depth = depth + 1;
    children.put(child.flow, child);
    return inserted;
  }

  @Override // from AbstractNode
  protected final boolean protected_removeChild(GlcNodeImpl child) {
    boolean removed = children.containsKey(child.flow);
    children.remove(child.flow);
    return removed;
  }

  @Override
  public Flow flow() {
    return flow;
  }

  @Override
  public StateTime stateTime() {
    return stateTime;
  }

  /** @return cost from root plus min cost to goal */
  @Override
  public Scalar merit() {
    return merit;
  }

  // function is only called by motion planners.
  // data structures that rely on the sorting by merit
  // may become invalid once the merit is set to a new value
  /* package */ @Override
  public void setMinCostToGoal(Scalar minCostToGoal) {
    merit = costFromRoot.add(minCostToGoal);
  }

  @Override
  public int depth() {
    return depth;
  }

  @Override
  public int reCalculateDepth() {
    depth = Nodes.toRoot(this).size() - 1;
    return depth; // as RootNode has depth 0 (NOT 1)
  }
}
