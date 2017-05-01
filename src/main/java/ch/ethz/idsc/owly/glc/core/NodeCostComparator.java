// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.Comparator;

import ch.ethz.idsc.tensor.Scalars;

/** compare two nodes based on {@link Node#cost()} */
/* package */ enum NodeCostComparator implements Comparator<Node> {
  instance;
  // ---
  @Override
  public int compare(Node o1, Node o2) {
    return Scalars.compare(o1.cost(), o2.cost());
  }
}
