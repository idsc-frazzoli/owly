// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.PriorityQueue;

/* package */ class DomainQueue extends PriorityQueue<GlcNode> {
  public DomainQueue() {
    super(NodeMeritComparator.INSTANCE);
  }
}
