// code by jl
package ch.ethz.idsc.owly.demo.twd;

import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2AbstractGoalManager;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.AbsSquared;
import ch.ethz.idsc.tensor.sca.Ramp;

/** Se2 goal region is not elliptic, therefore we implement {@link Region}
 * 
 * bapaden phd thesis: (6.4.1) */
// TODO consider making this a cost function instead of a goal interface
public class TwdMinCurvatureGoalManager extends Se2AbstractGoalManager {
  private final Scalar tolerance_xy;

  public TwdMinCurvatureGoalManager(Tensor center, Scalar tolerance_xy, Scalar tolerance_angle) {
    super(center, Tensors.of(tolerance_xy, tolerance_xy, tolerance_angle));
    this.tolerance_xy = tolerance_xy;
  }

  /** Curvature is changed angle over distance covered. */
  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    StateTime from = node.stateTime();
    StateTime end = trajectory.get(trajectory.size() - 1);
    Scalar dt = StateTimeTrajectories.timeIncrement(from, trajectory);
    // J(x,u) = (1+(delta(theta)/delta(position))²) * Ts
    Scalar angledef = TwdHelper.errorRotation(end.state(), from.state());
    return RealScalar.ONE.add(AbsSquared.FUNCTION.apply(angledef) //
        .divide(TwdHelper.errorPosition(from.state(), end.state()).add(RealScalar.ONE))) // if turnign on the place
        .multiply(dt);
  }

  // TODO only valid for assumption of maxSpeed = 1
  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(TwdHelper.errorPosition(x, center).subtract(tolerance_xy));
  }
}