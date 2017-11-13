// code by jph and jl
package ch.ethz.idsc.owly.demo.twd.glc;

import java.util.List;

import ch.ethz.idsc.owly.demo.twd.TwdAbstractGoalManager;
import ch.ethz.idsc.owly.demo.twd.TwdHelper;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
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

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    StateTime from = node.stateTime();
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  // TODO only valid for assumption of maxSpeed = 1
  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(TwdHelper.errorPosition(x, center).subtract(tolerance_xy));
  }
}