// code by jph
package ch.ethz.idsc.owly.math.state;

import java.util.Collection;

public interface StateTimeCollector {
  /** @return collection of accumulated {@link StateTime}s */
  Collection<StateTime> getMembers();
}
