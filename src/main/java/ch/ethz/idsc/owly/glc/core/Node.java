// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** glc specific node */
public class Node {
  /** flow is null for root node */
  private final Flow flow;
  private final Tensor x;
  private final Scalar time;
  private final Scalar cost;
  private final Scalar merit;
  private final Map<Flow, Node> children = new HashMap<>();
  /** parent is null for root node */
  private Node parent = null;
  /** depth == 0 for root node, otherwise depth > 0 */
  private int depth = 0;

  /** @param flow that got us to this Node from the parent
   * @param x
   * @param time
   * @param cost
   * @param e */
  public Node(Flow flow, Tensor x, Scalar time, Scalar cost, Scalar e) {
    this.flow = flow;
    this.x = x;
    this.time = time;
    this.cost = cost;
    this.merit = cost.add(e);
  }

  public void addChild(Node child) {
    child.parent = this;
    child.depth = depth + 1;
    children.put(child.flow, child);
  }

  public Tensor x() {
    return x.unmodifiable();
  }

  public Flow flow() {
    return flow;
  }

  public Scalar cost() {
    return cost;
  }

  public Scalar merit() {
    return merit;
  }

  public StateTime getStateTime() {
    return new StateTime(x, time);
  }

  public Node parent() {
    return parent;
  }

  public boolean isRoot() {
    return parent == null;
  }

  public int depth() {
    return depth;
  }

  @Override
  public String toString() {
    return "@" + x.toString() + " cost=" + cost + " merit=" + merit;
  }
}
