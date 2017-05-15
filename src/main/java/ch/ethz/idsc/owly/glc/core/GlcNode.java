// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.owly.data.tree.AbstractNode;
import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** glc specific node
 * 
 * immutable except for children, parent, and depth which are only modified in
 * {@link GlcNode#addChild(GlcNode)} */
public class GlcNode extends AbstractNode<GlcNode> implements StateCostNode {
  private final Map<Flow, GlcNode> children = new HashMap<>();
  /** flow is null for root node */
  private final Flow flow;
  private final StateTime stateTime;
  private final Scalar costFromRoot;
  private Scalar merit; // TODO find solution with final
  /** depth == 0 for root node, otherwise depth > 0 */
  private int depth = 0;

  /** @param flow that got us to this Node from the parent, or null when this Node is the root
   * @param x
   * @param time
   * @param costFromRoot
   * @param minCostToGoal */
  public GlcNode(Flow flow, StateTime stateTime, Scalar costFromRoot, Scalar minCostToGoal) {
    this.flow = flow;
    this.stateTime = stateTime;
    this.costFromRoot = costFromRoot;
    this.merit = costFromRoot.add(minCostToGoal);
  }

  @Override // from Node
  public Collection<GlcNode> children() {
    return children.values();
  }

  @Override // from StateCostNode
  public Scalar costFromRoot() {
    return costFromRoot;
  }

  @Override // from StateCostNode
  public Tensor state() {
    return stateTime.x();
  }

  @Override // from AbstractNode
  protected boolean protected_registerChild(GlcNode child) {
    boolean inserted = !children.containsKey(child.flow);
    child.depth = depth + 1;
    children.put(child.flow, child);
    return inserted;
  }

  public Flow flow() {
    return flow;
  }

  public StateTime stateTime() {
    return stateTime;
  }

  public Scalar merit() {
    return merit;
  }

  public void setMinCostToGoal(Scalar minCostToGoal) {
    merit = costFromRoot.add(minCostToGoal);
  }

  public int depth() {
    return depth;
  }

  @Deprecated
  public void printNodeState() {
    System.out.println("(" + this.stateTime.x() + ")");
  }
}
