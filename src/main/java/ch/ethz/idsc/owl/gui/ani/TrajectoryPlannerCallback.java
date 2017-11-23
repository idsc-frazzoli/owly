// code by jph
package ch.ethz.idsc.owl.gui.ani;

import java.util.List;

import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.rrts.core.RrtsPlanner;

/**
 * 
 */
// API not finalized
public interface TrajectoryPlannerCallback {
  /** @param head
   * @param trajectoryPlanner */
  void expandResult(List<TrajectorySample> head, TrajectoryPlanner trajectoryPlanner);

  void expandResult(List<TrajectorySample> head, RrtsPlanner rrtsPlanner, List<TrajectorySample> tail);
}
