// code by jph
package ch.ethz.idsc.owly.rrts.core;

import java.util.Collection;

import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface RrtsNode extends StateCostNode {
  static RrtsNode createRoot(Tensor state, Scalar cost) {
    return new RrtsNodeImpl(state, cost);
  }

  @Override // from Node
  RrtsNode parent();

  @Override // from Node
  Collection<? extends RrtsNode> children();

  RrtsNode connectTo(Tensor state, Scalar costFromRoot);

  void rewireTo(RrtsNode child, Scalar costFromParent);
}
