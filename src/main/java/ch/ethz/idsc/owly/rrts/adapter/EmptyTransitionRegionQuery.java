// code by jph
package ch.ethz.idsc.owly.rrts.adapter;

import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.owly.rrts.core.TransitionRegionQuery;

public enum EmptyTransitionRegionQuery implements TransitionRegionQuery {
  INSTANCE;
  // ---
  @Override
  public boolean isDisjoint(Transition transition) {
    return true;
  }
}
