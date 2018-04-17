// code by ynager and jph
package ch.ethz.idsc.owl.glc.std;

import java.util.List;

import ch.ethz.idsc.owl.glc.core.CostIncrementFunction;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;

public interface PlannerConstraint {
  /** parameters as in {@link CostIncrementFunction}
   * 
   * @param glcNode from which trajectory starts
   * @param trajectory
   * @param flow along which trajectory was computed
   * @return true if planner may create a new node */
  boolean isSatisfied(GlcNode glcNode, List<StateTime> trajectory, Flow flow);
}
