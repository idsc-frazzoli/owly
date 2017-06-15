// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;
import java.util.PriorityQueue;

/* package */ class DomainQueue extends PriorityQueue<GlcNode> implements Serializable {
  public DomainQueue() {
    super(NodeMeritComparator.INSTANCE);
  }
}
