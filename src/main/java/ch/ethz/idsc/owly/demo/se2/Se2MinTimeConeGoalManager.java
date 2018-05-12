// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.sca.Ramp;

/** min time cost function with decent heuristic */
public final class Se2MinTimeConeGoalManager extends Se2AbstractConeGoalManager {
  /** @param xya {px, py, angle}
   * @param semi half angular width of cone in the interval [0, pi/2]
   * @param tolerance in orientation
   * @param controls */
  public static GoalInterface create(Tensor xya, Scalar semi, Scalar tolerance, Collection<Flow> controls) {
    return new Se2MinTimeConeGoalManager(xya, semi, tolerance, controls).getGoalInterface();
  }
  // ---

  private final Scalar maxSpeed;
  private final Scalar maxTurning;

  private Se2MinTimeConeGoalManager(Tensor xya, Scalar semi, Scalar tolerance, Collection<Flow> controls) {
    super(xya, semi, tolerance);
    maxSpeed = Se2Controls.maxSpeed(controls);
    maxTurning = Se2Controls.maxTurning(controls);
  }

  @Override // from CostFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    // units: d_ax [m] / maxSpeed [m/s] -> time [s]
    return Ramp.of(Max.of(d_xy(tensor).divide(maxSpeed), d_angle(tensor).divide(maxTurning)));
  }
}
