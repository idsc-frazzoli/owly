// code by jph and jl
package ch.ethz.idsc.owly.demo.twd;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Ramp;

/** assumes max forward speed of == 1 */
public class TwdMinTimeGoalManager extends TwdAbstractGoalManager {
  public TwdMinTimeGoalManager(Tensor center, Scalar tolerance_xy, Scalar tolerance_angle) {
    super(center, tolerance_xy, tolerance_angle);
  }

  @Override // from CostFunction
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  // TODO only valid for assumption of maxSpeed = 1
  @Override // from CostFunction
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(TwdStateSpaceModel.errorPosition(x, center).subtract(tolerance_xy));
  }

  @Override
  public boolean hasHeuristic() {
    return true;
  }
}