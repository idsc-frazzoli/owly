// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import ch.ethz.idsc.owly.data.tree.AbstractNode;
import ch.ethz.idsc.owly.data.tree.Nodes;
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
  private Scalar merit;
  private volatile int hashCode;// TODO find solution with final
  /** depth == 0 for root node, otherwise depth > 0 */
  private int depth = 0;

  /** @param flow that got us to this Node from the parent, or null when this Node is the root
   * @param stateTime
   * @param costFromRoot
   * @param minCostToGoal */
  public GlcNode(Flow flow, StateTime stateTime, Scalar costFromRoot, Scalar minCostToGoal) {
    this.flow = flow;
    this.stateTime = stateTime;
    this.costFromRoot = costFromRoot;
    setMinCostToGoal(minCostToGoal);
  }

  @Override // from Node
  public Collection<GlcNode> children() {
    return children.values();
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
  protected boolean protected_registerChild(GlcNode child) {
    // boolean inserted = !children.containsKey(child.flow);
    child.depth = depth + 1;
    GlcNode former = children.put(child.flow, child);
    return former == null;
  }

  public Flow flow() {
    return flow;
  }

  public StateTime stateTime() {
    return stateTime;
  }

  /** @return cost from root plus min cost to goal */
  public Scalar merit() {
    return merit;
  }

  // function is only called by motion planners.
  // data structures that rely on the sorting by merit
  // may become invalid once the merit is set to a new value
  /* package */ void setMinCostToGoal(Scalar minCostToGoal) {
    merit = costFromRoot.add(minCostToGoal);
  }

  public int depth() {
    return depth;
  }

  /* package */ int reCalculateDepth() {
    this.depth = Nodes.toRoot(this).size() - 1;
    return depth;// as RootNode has depth 0 (NOT 1)
  }

  @Override
  public boolean equals(Object object) {
    if (object instanceof GlcNode) {
      GlcNode glcNode = (GlcNode) object;
      // TODO nicer solution then with null check (problem at root)
      if (flow == null && glcNode.flow == null)
        return stateTime.equals(glcNode.stateTime) && //
            costFromRoot.equals(glcNode.costFromRoot);
      if (flow != null && glcNode.flow != null)
        return stateTime.equals(glcNode.stateTime) && //
            costFromRoot.equals(glcNode.costFromRoot) && //
            flow.equals(glcNode.flow);
      // TODO workd with flow? as flow class has no equal?
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(stateTime, costFromRoot, flow);
  }
}
