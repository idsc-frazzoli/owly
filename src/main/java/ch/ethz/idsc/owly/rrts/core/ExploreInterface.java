// code by jph
package ch.ethz.idsc.owly.rrts.core;

import java.util.Optional;

// EXPERIMENTAL API no finalized
public interface ExploreInterface {
  /** @return non-empty optional if goal was inserted */
  Optional<RrtsNode> next();

  Optional<RrtsNode> getBest();
}
