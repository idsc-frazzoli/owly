// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.tensor.Tensor;

class Node {
  public Node parent;
  public final Map<Tensor, Node> children = new HashMap<>();
  public final Tensor x;
  public double time;
  public final double cost;
  public final double merit;
  public final Tensor u_idx;
  public int depth;

  public Node(Tensor tensor, double cost, double time, double e, Tensor u) {
    this.cost = cost;
    this.x = tensor;
    this.time = time;
    this.merit = cost + e;
    this.u_idx = u;
  }

  public void addChild(Node child, double expand_time) {
    Node _parent = this;
    child.parent = _parent;
    child.depth = _parent.depth + 1;
    child.time = _parent.time + expand_time;
    _parent.children.put(child.u_idx, child);
  }

  @Override
  public String toString() {
    return "@" + x.toString() + " cost=" + cost + " merit=" + merit;
  }

  public StateTime getStateTime() {
    return new StateTime(x, time);
  }
}
