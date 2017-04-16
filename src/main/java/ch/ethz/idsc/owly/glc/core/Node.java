// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.owly.util.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class Node {
  public Node parent;
  public final Map<Flow, Node> children = new HashMap<>();
  public final Tensor x;
  public Scalar time;
  public final Scalar cost;
  public final Scalar merit;
  /** u is null for root node */
  public final Flow u;
  public int depth;

  public Node(Tensor tensor, Scalar cost, Scalar time, Scalar e, Flow u) {
    this.cost = cost;
    this.x = tensor;
    this.time = time;
    this.merit = cost.add(e);
    this.u = u;
  }

  public void addChild(Node child, Scalar expand_time) {
    Node _parent = this;
    child.parent = _parent;
    child.depth = _parent.depth + 1;
    child.time = _parent.time.add(expand_time);
    _parent.children.put(child.u, child);
  }

  @Override
  public String toString() {
    return "@" + x.toString() + " cost=" + cost + " merit=" + merit;
  }

  public StateTime getStateTime() {
    return new StateTime(x, time);
  }
}
