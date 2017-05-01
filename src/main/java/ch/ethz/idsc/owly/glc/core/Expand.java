// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

public enum Expand {
  ;
  public static int maxSteps(ExpandInterface expandInterface, int expandLimit) {
    int expandCount = 0;
    while (expandCount++ < expandLimit) {
      Node node = expandInterface.pollNext();
      if (node == null) // queue is empty
        break;
      expandInterface.expand(node);
      if (expandInterface.getBest() != null) // found node in goal region
        break;
    }
    return expandCount;
  }

  public static void maxDepth(ExpandInterface expandInterface, int depthLimit) {
    while (true) {
      Node node = expandInterface.pollNext();
      if (node == null) // queue is empty
        break;
      expandInterface.expand(node);
      if (expandInterface.getBest() != null) // found node in goal region
        break;
      if (depthLimit < node.depth())
        break;
    }
  }
}
