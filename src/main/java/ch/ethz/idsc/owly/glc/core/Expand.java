// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

public enum Expand {
  ;
  public static int maxSteps(ExpandInterface trajectoryPlanner, int expandLimit) {
    int expandCount = 0;
    while (expandCount++ < expandLimit) {
      Node current_node = trajectoryPlanner.pollNext();
      if (current_node == null) // queue is empty
        break;
      trajectoryPlanner.expand(current_node);
      if (trajectoryPlanner.getBest() != null)
        break;
    }
    return expandCount;
  }

  public static void maxDepth(ExpandInterface trajectoryPlanner, int depthLimit) {
    while (true) {
      Node current_node = trajectoryPlanner.pollNext();
      if (current_node == null) // queue is empty
        break;
      trajectoryPlanner.expand(current_node);
      if (trajectoryPlanner.getBest() != null) // found goal node
        break;
      if (depthLimit < current_node.depth())
        break;
    }
  }
}
