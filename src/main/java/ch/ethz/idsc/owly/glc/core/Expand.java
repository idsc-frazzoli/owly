// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Optional;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
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
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) { // queue is empty
        System.out.println("*** Queue is empty -- No Goal was found ***");
        break;
      }
      expandInterface.expand(next.get());
      if (expandInterface.getBest().isPresent()) // found node in goal region
        break;
    }
    // no printout here, since expand limit can deliberately set to a low number for animation
    // see Se2rExpandDemo
    return expandCount;
  }

  /** total number of expands are bounded by expandLimit & depthLimit
   * 
   * @param expandInterface
   * @param expandLimit
   * @param depthLimit
   * @return */
  public static int maxSteps(ExpandInterface expandInterface, int expandLimit, int depthLimit) {
    int expandCount = 0;
    while (expandCount++ < expandLimit) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) { // queue is empty
        System.out.println("*** Queue is empty -- No Goal was found ***");
        break;
      }
      expandInterface.expand(next.get());
      if (expandInterface.getBest().isPresent()) // found node in goal region
        break;
      if (depthLimit < next.get().depth()) {
        System.err.println("*** DepthLimit reached -- No Goal was found ***");
        break;
      }
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
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) { // queue is empty
        System.err.println("**** Queue is empty -- No Goal was found");// queue is empty
        break;
      }
      expandInterface.expand(next.get());
      if (expandInterface.getBest().isPresent()) // found node in goal region
        break;
      if (depthLimit < next.get().depth()) {
        System.err.println("*** DepthLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }

  /** expands until the time of the running algorithm exceeds the maxTime
   * or a goal was found
   * @param expandInterface
   * @param timeLimit TimeLimit of expandfunction in [s] */
  public static int maxTime(ExpandInterface expandInterface, Scalar timeLimit) {
    long tic = System.nanoTime();
    timeLimit = timeLimit.multiply(RealScalar.of(1e9));
    int expandCount = 0;
    while (true) {
      expandCount++;
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) {
        System.err.println("**** Queue is empty -- No Goal was found");// queue is empty
        break;
      }
      expandInterface.expand(next.get());
      long toc = System.nanoTime();
      if (expandInterface.getBest().isPresent()) { // found node in goal region
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

  /** expands until the time of the running algorithm exceeds time or depthlimit is reached
   * 
   * @param expandInterface
   * @param time Time of expandfunction in [s] */
  public static int constTime(ExpandInterface expandInterface, Scalar time, int depthLimit) {
    long tic = System.nanoTime();
    time = time.multiply(RealScalar.of(1e9));
    int expandCount = 0;
    while (true) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) {
        System.err.println("**** Queue is empty ****");// queue is empty
        break;
      }
      expandInterface.expand(next.get());
      expandCount++;
      long toc = System.nanoTime();
      if (Scalars.lessThan(time, RealScalar.of(toc - tic))) {
        System.out.println("***Planned for " + time.multiply(RealScalar.of(1e-9)) + "s ***");
        break;
      }
      if (depthLimit < next.get().depth()) {
        System.err.println("*** DepthLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }
}
