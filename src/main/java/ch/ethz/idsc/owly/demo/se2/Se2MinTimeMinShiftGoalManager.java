// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owly.data.DontModify;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.sca.Ramp;

/** min time cost function with decent heuristic
 * penalizes switching between forwards and backwards driving
 * 
 * The cost does not account for curvature. */
@DontModify
public final class Se2MinTimeMinShiftGoalManager extends Se2AbstractGoalManager {
  private static final Scalar SHIFT_PENALTY = RealScalar.of(.4);

  public static GoalInterface create(Tensor goal, Tensor radiusVector, Collection<Flow> controls) {
    return new Se2MinTimeMinShiftGoalManager(goal, radiusVector, controls).getGoalInterface();
  }

  // ---
  private final Scalar maxSpeed;
  private final Scalar maxTurning;

  private Se2MinTimeMinShiftGoalManager(Tensor goal, Tensor radiusVector, Collection<Flow> controls) {
    super(goal, radiusVector);
    maxSpeed = Se2Controls.maxSpeed(controls);
    maxTurning = Se2Controls.maxTurning(controls);
  }

  @Override // from CostFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    Flow ante = glcNode.flow(); // == null if glcNode is root
    boolean steady = Objects.nonNull(ante) ? ante.getU().Get(1).equals(flow.getU().Get(1)) : true;
    Scalar penalty = steady ? RealScalar.ZERO : SHIFT_PENALTY;
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory).add(penalty);
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    return Ramp.of(Max.of( //
        d_xy(tensor).subtract(radiusSpace()).divide(maxSpeed), //
        d_angle(tensor).subtract(radiusAngle()).divide(maxTurning) //
    ));
  }
}
