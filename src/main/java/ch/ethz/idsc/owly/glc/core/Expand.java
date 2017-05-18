// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

/** class contains static utility function that operate on instances of the
 * {@link ExpandInterface} */
public enum Expand {
  ;
  /** total number of expands are bounded by expandLimit
   * 
   * @param expandInterface
   * @param expandLimit
   * @return */
  public static int maxSteps(ExpandInterface expandInterface, int expandLimit) {
    int expandCount = 0;
    while (expandCount++ < expandLimit) {
      GlcNode node = expandInterface.pollNext();
      if (node == null) { // queue is empty
        System.out.println("*** Queue is empty -- No Goal was found ***");
        break;
      }
      expandInterface.expand(node);
      if (expandInterface.getBest() != null) // found node in goal region
        break;
    }
    // no printout here, since
    // expand limit can deliberately set to a low number for animation
    return expandCount;
  }

  /** expands until the depth of the polled node exceeds given depthLimit
   * 
   * @param expandInterface
   * @param depthLimit */
  public static int maxDepth(ExpandInterface expandInterface, int depthLimit) {
    int expandCount = 0;
    while (true) {
      expandCount++;
      GlcNode node = expandInterface.pollNext();
      if (node == null) // queue is empty
        break;
      expandInterface.expand(node);
      if (expandInterface.getBest() != null) // found node in goal region
        break;
      if (depthLimit < node.depth()) {
        System.out.println("*** DepthLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }
}
