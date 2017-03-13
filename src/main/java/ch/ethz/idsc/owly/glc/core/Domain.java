// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.PriorityQueue;

class Domain {
  private Node label = null;
  PriorityQueue<Node> candidates = new PriorityQueue<>(NodeCostComparator.instance);

  public boolean empty() {
    return label == null;
  }

  public double getCost() {
    return empty() ? Double.POSITIVE_INFINITY : label.cost;
  }

  public void setLabel(Node node) {
    label = node;
  }
}
