// code by ynager
package ch.ethz.idsc.owl.glc.core;

import java.util.List;

import ch.ethz.idsc.owl.math.state.StateTime;

public interface Constraint {
  /** @param glcNode from which trajectory starts
   * @param parent node of glcNode
   * @param trajectory
   * @param flow along which trajectory was computed
   * @return true if constraint along trajectory is satisfied */
  boolean isSatisfied(GlcNode glcNode, GlcNode parentNode, List<StateTime> trajectory);
}
