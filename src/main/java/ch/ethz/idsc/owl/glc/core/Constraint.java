// code by ynager
package ch.ethz.idsc.owl.glc.core;

import java.util.List;

import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;

public interface Constraint {
  /** @param glcNode from which trajectory starts
   * @param trajectory
   * @param flow along which trajectory was computed
   * @return true if constraint satisfied, false otherwise */
  boolean isSatisfied(GlcNode glcNode, List<StateTime> trajectory, Flow flow);
}
