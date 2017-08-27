// code by jph and jl
package ch.ethz.idsc.owly.glc.core;

import java.util.Optional;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.data.Stopwatch;
import ch.ethz.idsc.tensor.Scalar;

/** class contains static functions that operate on instances of {@link ExpandInterface}
 * 
 * The expansion of the following planners can be controlled using the functions:
 * <ul>
 * <li>{@link StandardTrajectoryPlanner},
 * <li>{@link SimpleAnyTrajectoryPlanner},
 * <li>{@link OptimalAnyTrajectoryPlanner}
 * </ul> */
public enum Expand {
  ;
  /** @param expandInterface
   * @param expandLimit
   * @return number times function {@link ExpandInterface#expand(GlcNode)} was invoked */
  public static int maxSteps(ExpandInterface expandInterface, int expandLimit) {
    return maxSteps(expandInterface, expandLimit, () -> true);
  }

  /** planner aborts if isContinued supplies false
   * 
   * @param expandInterface
   * @param expandLimit
   * @param isContinued
   * @return number times function {@link ExpandInterface#expand(GlcNode)} was invoked */
  public static int maxSteps(ExpandInterface expandInterface, int expandLimit, Supplier<Boolean> isContinued) {
    int expandCount = 0;
    while (expandCount < expandLimit) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) { // queue is empty
        System.out.println("*** Queue is empty -- No Goal was found ***");
        break;
      }
      // System.out.println("expand "+next.get().stateTime().toInfoString());
      expandInterface.expand(next.get());
      ++expandCount;
      if (expandInterface.getBest().isPresent()) // found node in goal region
        break;
      if (!isContinued.get())
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
   * @return number times function {@link ExpandInterface#expand(GlcNode)} was invoked */
  public static int maxSteps(ExpandInterface expandInterface, int expandLimit, int depthLimit) {
    int expandCount = 0;
    while (expandCount < expandLimit) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) { // queue is empty
        System.out.println("*** Queue is empty -- No Goal was found ***");
        break;
      }
      expandInterface.expand(next.get());
      ++expandCount;
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
   * @param depthLimit
   * @return number times function {@link ExpandInterface#expand(GlcNode)} was invoked */
  public static int maxDepth(ExpandInterface expandInterface, int depthLimit) {
    System.out.println("*** EXPANDING ***");
    int expandCount = 0;
    while (true) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) { // queue is empty
        System.err.println("**** Queue is empty -- No Goal was found");// queue is empty
        break;
      }
      expandInterface.expand(next.get());
      ++expandCount;
      if (expandInterface.getBest().isPresent()) // found node in goal region
        break;
      if (depthLimit < next.get().depth()) {
        System.err.println("*** DepthLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }

  /** expands until the time of the running algorithm exceeds the maxTime or a goal was found
   * 
   * @param expandInterface
   * @param timeLimit of expandfunction in [s]
   * @return number times function {@link ExpandInterface#expand(GlcNode)} was invoked */
  public static int maxTime(ExpandInterface expandInterface, Scalar timeLimit) {
    System.out.println("*** EXPANDING ***");
    Stopwatch stopwatch = Stopwatch.started();
    final double time = timeLimit.number().doubleValue();
    int expandCount = 0;
    while (true) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) {
        System.err.println("**** Queue is empty -- No Goal was found"); // queue is empty
        break;
      }
      expandInterface.expand(next.get());
      ++expandCount;
      if (expandInterface.getBest().isPresent()) { // found node in goal region
        stopwatch.stop();
        System.out.println("after " + stopwatch.display_seconds() + "s");
        break;
      }
      if (time < stopwatch.display_seconds()) {
        System.out.println("*** TimeLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }

  /** expands until the time of the running algorithm exceeds time or depthlimit is reached
   * 
   * @param expandInterface
   * @param time Time of expandfunction in [s]
   * @return number times function {@link ExpandInterface#expand(GlcNode)} was invoked */
  public static int constTime(ExpandInterface expandInterface, Scalar _time, int depthLimit) {
    System.out.println("*** EXPANDING ***");
    Stopwatch stopwatch = Stopwatch.started();
    final double time = _time.number().doubleValue();
    int expandCount = 0;
    while (true) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) {
        System.err.println("**** Queue is empty ****"); // queue is empty
        break;
      }
      expandInterface.expand(next.get());
      ++expandCount;
      if (time < stopwatch.display_seconds()) {
        System.out.println("***Planned for " + _time + "s ***");
        break;
      }
      if (depthLimit < next.get().depth()) {
        System.err.println("*** DepthLimit reached -- No Goal was found ***");
        break;
      }
    }
    return expandCount;
  }

  /** total number of expands are bounded by expandLimit
   * 
   * @param expandInterface
   * @param expandLimit
   * @return number times function {@link ExpandInterface#expand(GlcNode)} was invoked */
  public static int steps(ExpandInterface expandInterface, int expandLimit) {
    int expandCount = 0;
    while (expandCount < expandLimit) {
      Optional<GlcNode> next = expandInterface.pollNext();
      if (!next.isPresent()) { // queue is empty
        System.out.println("*** Queue is empty -- No Goal was found ***");
        break;
      }
      expandInterface.expand(next.get());
      ++expandCount;
    }
    // no printout here, since expand limit can deliberately set to a low number for animation
    // see Se2rExpandDemo
    return expandCount;
  }
}
