// code by ynager
package ch.ethz.idsc.owl.glc.core;

import java.io.Serializable;
import java.util.List;

import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;

public interface Constraint extends Serializable{
  
  /** @param glcNode from which trajectory starts
   * @param trajectory
   * @param flow along which trajectory was computed
   * @return true if constraint along trajectory is satisfied*/
  boolean isSatisfied(GlcNode glcNode, GlcNode prev, List<StateTime> trajectory, Flow flow);
}
