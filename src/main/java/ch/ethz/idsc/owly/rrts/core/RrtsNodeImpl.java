// code by jph
package ch.ethz.idsc.owly.rrts.core;

import ch.ethz.idsc.owly.data.tree.SetNode;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/* package */ class RrtsNodeImpl extends SetNode<RrtsNode> implements RrtsNode {
  private final Tensor state;
  private Scalar costFromRoot;

  /* package */ RrtsNodeImpl(Tensor state, Scalar costFromRoot) {
    this.state = state;
    this.costFromRoot = costFromRoot;
  }

  @Override
  public RrtsNode connectTo(Tensor state, Scalar costFromRoot) {
    RrtsNodeImpl leaf = new RrtsNodeImpl(state, costFromRoot);
    this.insertEdgeTo(leaf);
    return leaf;
  }

  @Override
  public void rewireTo(RrtsNode child, Scalar costFromParent) {
    child.parent().removeEdgeTo(child);
    insertEdgeTo(child);
    final Scalar nodeCostFromRoot = costFromRoot().add(costFromParent);
    // the condition of cost reduction is not strictly necessary
    if (!Scalars.lessThan(nodeCostFromRoot, child.costFromRoot()))
      throw new RuntimeException();
    _propagate(child, nodeCostFromRoot);
    ((RrtsNodeImpl) child).costFromRoot = nodeCostFromRoot;
  }

  private static void _propagate(RrtsNode node, Scalar nodeCostFromRoot) {
    for (RrtsNode _child : node.children()) {
      RrtsNodeImpl child = (RrtsNodeImpl) _child;
      final Scalar costFromParent = child.costFromRoot().subtract(node.costFromRoot());
      Scalar newCostFromRoot = nodeCostFromRoot.add(costFromParent);
      _propagate(child, newCostFromRoot);
      child.costFromRoot = newCostFromRoot;
    }
  }

  @Override
  public Tensor state() {
    return state;
  }

  @Override
  public Scalar costFromRoot() {
    return costFromRoot;
  }
}