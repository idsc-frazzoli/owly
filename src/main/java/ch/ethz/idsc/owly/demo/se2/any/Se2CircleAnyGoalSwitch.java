// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.awt.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import ch.ethz.idsc.owl.glc.adapter.MultiCostGoalAdapter;
import ch.ethz.idsc.owl.glc.any.AbstractAnyTrajectoryPlanner;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.glc.par.Parameters;
import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.demo.se2.Se2LateralAcceleration;
import ch.ethz.idsc.owly.demo.se2.Se2MinTimeGoalManager;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Cos;
import ch.ethz.idsc.tensor.sca.Sin;

enum Se2CircleAnyGoalSwitch {
  ;
  public static boolean switchToNextCircularGoal(AbstractAnyTrajectoryPlanner trajectoryPlanner, int iter, Collection<Flow> controls) {
    return switchToNextCircularGoal(trajectoryPlanner, iter, controls, null);
  }

  public static boolean switchToNextCircularGoal(//
      AbstractAnyTrajectoryPlanner trajectoryPlanner, //
      int iter, //
      Collection<Flow> controls, //
      Parameters parameters) {
    Scalar stepsPerCircle = RealScalar.of(4);
    Scalar circleRadius = RealScalar.of(3);
    Tensor goal = null;
    Tensor radiusVector = Tensors.of(DoubleScalar.of(0.2), DoubleScalar.of(0.2), Degree.of(15));
    
    ArrayList<StateTime> goalList = new ArrayList<StateTime>();
    goalList.add(new StateTime(goal, RealScalar.ZERO));
    
    do {
      Scalar goalAngle = RealScalar.of(2 * Math.PI).divide(stepsPerCircle).multiply(RealScalar.of(iter)).negate();
      goal = Tensors.of(Cos.of(goalAngle).multiply(circleRadius), //
          Sin.of(goalAngle).multiply(circleRadius), goalAngle.subtract(RealScalar.of(Math.PI * 0.5)));
    // } while (trajectoryPlanner.getObstacleQuery().isMember(new StateTime(goal, RealScalar.ZERO)));
    } while (trajectoryPlanner.getPlannerConstraint().isSatisfied(null, goalList, null)); // YNAGER TODO test changes
    GoalInterface se2GoalManager = MultiCostGoalAdapter.of( //
        Se2MinTimeGoalManager.create(goal, radiusVector, controls), //
        Arrays.asList(Se2LateralAcceleration.COSTFUNCTION));
    if (parameters != null) { // changeGoal can be conducted quicker, due to GoalHint
      Tensor maxChange = Tensors.of(RealScalar.ONE, RealScalar.ONE, Degree.of(45));
      Tensor possibleGoalReachabilityRegionRadius = radiusVector.add(maxChange);
      Se2NoHeuristicGoalManager possibleGoalReachabilityRegion = new Se2NoHeuristicGoalManager(goal, possibleGoalReachabilityRegionRadius);
      return trajectoryPlanner.changeToGoal(se2GoalManager, possibleGoalReachabilityRegion);
    }
    return trajectoryPlanner.changeToGoal(se2GoalManager);
  }
}
