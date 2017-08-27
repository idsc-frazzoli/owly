// code by jph
package ch.ethz.idsc.owly.gui.ani;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
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
