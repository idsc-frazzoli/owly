package ch.ethz.idsc.owly.demo.se2.glc;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2MinCurvatureGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2Utils;
import ch.ethz.idsc.owly.glc.core.AbstractAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

public class Se2CircleAnyDemo {
  public static boolean switchToNextCircularGoal(AbstractAnyTrajectoryPlanner trajectoryPlanner, int iter) {
    List<StateTime> goalStateList = new ArrayList<>();
    Scalar stepsPerCircle = RealScalar.of(4);
    Scalar circleRadius = RealScalar.of(3);
    Tensor goal = null;
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), Se2Utils.DEGREE(15));
    do {
      goalStateList.clear();
      Scalar goalAngle = RealScalar.of(2 * Math.PI).divide(stepsPerCircle).multiply(RealScalar.of(iter)).negate();
      goal = Tensors.of(Cos.of(goalAngle).multiply(circleRadius), //
          Sin.of(goalAngle).multiply(circleRadius), goalAngle.subtract(RealScalar.of(Math.PI * 0.5)));
      StateTime goalState = new StateTime(goal, RealScalar.ZERO);
      goalStateList.add(goalState);
    } while (!trajectoryPlanner.getObstacleQuery().isDisjoint(goalStateList));
    Se2MinCurvatureGoalManager se2GoalManager = new Se2MinCurvatureGoalManager(goal, radiusVector);
    boolean goalFound = trajectoryPlanner.changeGoal(se2GoalManager.getGoalInterface());
    return goalFound;
  }
}
