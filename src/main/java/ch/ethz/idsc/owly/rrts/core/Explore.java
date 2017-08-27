// code by jph
package ch.ethz.idsc.owly.rrts.core;

import java.util.Optional;

import ch.ethz.idsc.owly.glc.core.Expand;

/** analogous to
 * @see Expand */
public enum Explore {
  ;
  public static int maxSteps(ExploreInterface exploreInterface, int exploreLimit) {
    int exploreCount = 0;
    while (exploreCount < exploreLimit) {
      Optional<RrtsNode> optional = exploreInterface.next();
      ++exploreCount;
      if (optional.isPresent())
        break;
    }
    return exploreCount;
  }

  public static int constantSteps(ExploreInterface exploreInterface, int exploreLimit) {
    for (int iters = 0; iters < exploreLimit; ++iters)
      exploreInterface.next();
    return exploreLimit;
  }
}
