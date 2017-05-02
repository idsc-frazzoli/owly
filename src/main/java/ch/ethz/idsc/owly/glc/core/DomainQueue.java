// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.PriorityQueue;

/* package */ class DomainQueue extends PriorityQueue<Node> {
  public DomainQueue(Node node) {
    super(NodeCostComparator.instance);
    add(node);
  }
}
