// code by jl
package ch.ethz.idsc.owly.demo.twd;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Ramp;

/** Se2 goal region is not elliptic, therefore we implement {@link Region}
 * 
 * bapaden phd thesis: (6.4.1) */
public class TwdMinCurvatureGoalManager extends TwdAbstractGoalManager {
  public TwdMinCurvatureGoalManager(Tensor center, Scalar tolerance_xy, Scalar tolerance_angle) {
    super(center, tolerance_xy, tolerance_angle);
  }

  /** Curvature is changed angle over distance covered. */
  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    StateTime from = node.stateTime();
    StateTime end = trajectory.get(trajectory.size() - 1);
    // J(x,u) = (1+(delta(theta)/delta(position))²) * Ts
    // TODO JONAS 1) instead of "end.state().Get(2).subtract(from.state().Get(2))" -> use
    // ... TwdStateSpaceModel.errorRotation(...)
    // TODO JONAS 2) instead of Power.of(...,2) -> use
    // ... AbsSquared.of(...) for faster result
    // TwdStateSpaceModel.errorRotation(end.state(), from.state());
    return (RealScalar.ONE.add(Power.of(end.state().Get(2).subtract(from.state().Get(2)), 2) //
        .divide(TwdStateSpaceModel.errorPosition(from.state(), end.state()).add(RealScalar.ONE))))// if turnign on the place
            .multiply(StateTimeTrajectories.timeIncrement(from, trajectory));
  }

  // TODO only valid for assumption of maxSpeed = 1
  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(TwdStateSpaceModel.errorPosition(x, center).subtract(tolerance_xy));
  }
}