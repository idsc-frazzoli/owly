// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Ramp;

/**
 * 
 */
public final class Se2MinCurvatureGoalManager extends Se2AbstractGoalManager {
  public Se2MinCurvatureGoalManager(Tensor center, Tensor radiusVector) {
    super(center, radiusVector);
  }

  @Override // from CostFunction
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    StateTime from = node.stateTime();
    return RealScalar.ONE.add(Power.of(flow.getU().Get(1), 2)) //
        .multiply(StateTimeTrajectories.timeIncrement(from, trajectory));
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    // TODO JONAS how come the d_angle can be used here but not in Se2MinDistCurvGoalManager ?
    return Ramp.of(Max.of( //
        d_xy(tensor).subtract(radiusSpace()), //
        d_angle(tensor).subtract(radiusAngle())));
  }
}
