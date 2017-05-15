// code by jph
package ch.ethz.idsc.owly.data.tree;

import java.util.Collection;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface StateCostNode extends Node {
  @Override // from Node
  StateCostNode parent();

  @Override // from Node
  Collection<? extends StateCostNode> children();

  /** @return cost from root to this node */
  Scalar costFromRoot();

  /** @return state associated to this node */
  Tensor state();
}
