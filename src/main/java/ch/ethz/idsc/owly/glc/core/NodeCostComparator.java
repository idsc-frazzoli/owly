// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Comparator;

import ch.ethz.idsc.tensor.Scalars;

class NodeCostComparator implements Comparator<Node> {
  static final NodeCostComparator instance = new NodeCostComparator();

  @Override
  public int compare(Node o1, Node o2) {
    return -Scalars.compare(o1.cost, o2.cost);
  }
}
