// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;

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
    // no printout here, since expand limit can deliberately set to a low number for animation
    // see Se2rExpandDemo
    return expandCount;
  }

  /** expands until the depth of the polled node exceeds given depthLimit
   * 
   * @param expandInterface
   * @param depthLimit */
  public static int maxDepth(ExpandInterface expandInterface, int depthLimit) {
    System.out.println("Expanding");
    int expandCount = 0;
    while (true) {
      expandCount++;
      GlcNode node = expandInterface.pollNext();
      if (node == null) {
        System.err.println("**** Queue is empty -- No Goal was found");// queue is empty
        break;
      }
      expandInterface.expand(node);
      if (expandInterface.getBest() != null) // found node in goal region
        break;
      if (depthLimit < node.depth()) {
        System.err.println("*** DepthLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }

  /** expands until the time of the running algorithm exceeds the maxTime
   * 
   * @param expandInterface
   * @param timeLimit TimeLimit of expandfunction in [s] */
  public static int maxTime(ExpandInterface expandInterface, RealScalar timeLimit) {
    long tic = System.nanoTime();
    timeLimit = (RealScalar) timeLimit.multiply(RealScalar.of(1e9));
    int expandCount = 0;
    while (true) {
      expandCount++;
      GlcNode node = expandInterface.pollNext();
      if (node == null) {
        System.err.println("**** Queue is empty -- No Goal was found");// queue is empty
        break;
      }
      expandInterface.expand(node);
      long toc = System.nanoTime();
      if (expandInterface.getBest() != null) { // found node in goal region
        System.out.println("after " + (toc - tic) * 1e-9 + "s");
        break;
      }
      if (Scalars.lessThan(timeLimit, RealScalar.of(toc - tic))) {
        System.out.println("*** TimeLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }
}
