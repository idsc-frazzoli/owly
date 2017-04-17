// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;

public class Domain {
  private Node label = null;

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

  public boolean takesOffer(Node new_arc) {
    return Scalars.lessThan(new_arc.cost, getCost());
  }
}
