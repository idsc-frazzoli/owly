// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.PriorityQueue;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

public class Domain {
  private Node label = null;
  @Deprecated
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

  public Node getLabel() {
    return label;
  }
}
