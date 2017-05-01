// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

public enum Expand {
  ;
  public static int maxSteps(ExpandInterface expandInterface, int expandLimit) {
    int expandCount = 0;
    while (expandCount++ < expandLimit) {
      Node current_node = expandInterface.pollNext();
      if (current_node == null) // queue is empty
        break;
      expandInterface.expand(current_node);
      if (expandInterface.getBest() != null)
        break;
    }
    return expandCount;
  }

  public static void maxDepth(ExpandInterface expandInterface, int depthLimit) {
    while (true) {
      Node current_node = expandInterface.pollNext();
      if (current_node == null) // queue is empty
        break;
      expandInterface.expand(current_node);
      if (expandInterface.getBest() != null) // found goal node
        break;
      if (depthLimit < current_node.depth())
        break;
    }
  }
}
