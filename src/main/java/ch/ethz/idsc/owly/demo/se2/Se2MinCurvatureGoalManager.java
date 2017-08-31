// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.SignedCurvature2D;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.sca.Floor;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Ramp;

/**
 * 
 */
public final class Se2MinCurvatureGoalManager extends Se2GoalRegion {
  public Se2MinCurvatureGoalManager(Tensor center, Tensor radiusVector) {
    super(center, radiusVector);
  }

  @Override // from CostFunction
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    // TODO JONAS complicated function probably can be simplified by using flow
    StateTime from = node.stateTime();
    int endIndex = trajectory.size() - 1;
    if (endIndex < 3) // can not calculated curvature with 2 points
      throw new RuntimeException();
    int middleIndex = Floor.of(RealScalar.of(endIndex / 2)).Get().number().intValue();
    List<Integer> indices1 = new ArrayList<Integer>();
    List<Integer> indices2 = new ArrayList<Integer>();
    indices1.add(0);
    indices2.add(2);
    Tensor a = from.state().block(indices1, indices2);
    Tensor b = trajectory.get(middleIndex).state().block(indices1, indices2);
    Tensor c = trajectory.get(endIndex).state().block(indices1, indices2);
    Scalar curvature = SignedCurvature2D.of(a, b, c);
    return RealScalar.ONE.add(Power.of(curvature.abs(), 2)) //
        .multiply(StateTimeTrajectories.timeIncrement(from, trajectory));
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    return Ramp.of(Max.of( //
        d_xy(tensor).subtract(radiusSpace()), //
        d_angle(tensor).subtract(radiusAngle())));
  }
}
