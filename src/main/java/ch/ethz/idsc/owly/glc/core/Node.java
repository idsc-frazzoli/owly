// code by bapaden and jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;

/** glc specific node
 * 
 * immutable except for children, parent, and depth which are only modified in
 * {@link Node#addChild(Node)} */
public class Node {
  /** flow is null for root node */
  private final Flow flow;
  private final StateTime stateTime;
  private final Scalar cost;
  private final Scalar merit;
  private final Map<Flow, Node> children = new HashMap<>();
  /** parent is null for root node */
  private Node parent = null;
  /** depth == 0 for root node, otherwise depth > 0 */
  private int depth = 0;

  /** @param flow that got us to this Node from the parent, or null when this Node is the root
   * @param x
   * @param time
   * @param cost
   * @param minCostToGoal */
  public Node(Flow flow, StateTime stateTime, Scalar cost, Scalar minCostToGoal) {
    this.flow = flow;
    this.stateTime = stateTime;
    this.cost = cost;
    this.merit = cost.add(minCostToGoal);
  }

  public void addChild(Node child) {
    child.parent = this;
    child.depth = depth + 1;
    children.put(child.flow, child);
  }

  public Flow flow() {
    return flow;
  }

  public StateTime stateTime() {
    return stateTime;
  }

  public Scalar cost() {
    return cost;
  }

  public Scalar merit() {
    return merit;
  }

  public Node parent() {
    return parent;
  }

  public Map<Flow, Node> children() {
    return children;
  }

  public boolean isRoot() {
    return parent == null;
  }

  public int depth() {
    return depth;
  }

  public void printNodeState() {
    System.out.println("("+ this.stateTime.x()+")");
    return;
    
  }
}
