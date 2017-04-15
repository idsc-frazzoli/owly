// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.PriorityQueue;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

class Domain {
  private Node label = null;
  PriorityQueue<Node> candidates = new PriorityQueue<>(NodeCostComparator.instance);

  public boolean empty() {
    return label == null;
  }

  public Scalar getCost() {
    return empty() ? RealScalar.POSITIVE_INFINITY : label.cost;
  }

  public void setLabel(Node node) {
    label = node;
  }
}
