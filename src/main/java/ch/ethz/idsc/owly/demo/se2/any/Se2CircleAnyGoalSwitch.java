// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.ArrayList;
import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2MinCurvatureGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2NoHeuristicGoalManager;
import ch.ethz.idsc.owly.glc.adapter.Parameters;
import ch.ethz.idsc.owly.glc.core.AbstractAnyTrajectoryPlanner;
import ch.ethz.idsc.owly.math.RotationUtils;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

enum Se2CircleAnyGoalSwitch {
  ;
  public static boolean switchToNextCircularGoal(AbstractAnyTrajectoryPlanner trajectoryPlanner, int iter) {
    return switchToNextCircularGoal(trajectoryPlanner, iter, null);
  }

  public static boolean switchToNextCircularGoal(AbstractAnyTrajectoryPlanner trajectoryPlanner, int iter, Parameters parameters) {
    List<StateTime> goalStateList = new ArrayList<>();
    Scalar stepsPerCircle = RealScalar.of(4);
    Scalar circleRadius = RealScalar.of(3);
    Tensor goal = null;
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), RotationUtils.DEGREE(15));
    do {
      goalStateList.clear();
      Scalar goalAngle = RealScalar.of(2 * Math.PI).divide(stepsPerCircle).multiply(RealScalar.of(iter)).negate();
      goal = Tensors.of(Cos.of(goalAngle).multiply(circleRadius), //
          Sin.of(goalAngle).multiply(circleRadius), goalAngle.subtract(RealScalar.of(Math.PI * 0.5)));
      StateTime goalState = new StateTime(goal, RealScalar.ZERO);
      goalStateList.add(goalState);
    } while (!trajectoryPlanner.getObstacleQuery().isDisjoint(goalStateList));
    Se2MinCurvatureGoalManager se2GoalManager = new Se2MinCurvatureGoalManager(goal, radiusVector);
    if (parameters != null) { // changeGoal can be conducted quicker, due to GoalHint
      Tensor maxChange = Tensors.of(RealScalar.ONE, RealScalar.ONE, RotationUtils.DEGREE(45));
      Tensor possibleGoalReachabilityRegionRadius = radiusVector.add(maxChange);
      Se2NoHeuristicGoalManager possibleGoalReachabilityRegion = new Se2NoHeuristicGoalManager(goal, possibleGoalReachabilityRegionRadius);
      return trajectoryPlanner.changeToGoal(se2GoalManager.getGoalInterface(), possibleGoalReachabilityRegion);
    } else {
      return trajectoryPlanner.changeToGoal(se2GoalManager.getGoalInterface());
    }
  }
}
