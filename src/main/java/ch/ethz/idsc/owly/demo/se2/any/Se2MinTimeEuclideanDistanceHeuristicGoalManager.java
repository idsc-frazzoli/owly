// code by jl
package ch.ethz.idsc.owly.demo.se2.any;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.demo.se2.Se2AbstractGoalManager;
import ch.ethz.idsc.owly.demo.se2.Se2Controls;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Ramp;

/** Nonholonomic Wheeled Robot
 * 
 * bapaden phd thesis: (5.5.13) */
// TODO JONAS use Se2MinTimeGoalManager instead
/* package */ class Se2MinTimeEuclideanDistanceHeuristicGoalManager extends Se2AbstractGoalManager {
  private final Scalar maxSpeed;

  public Se2MinTimeEuclideanDistanceHeuristicGoalManager(Tensor goal, Tensor radiusVector, Collection<Flow> controls) {
    super(goal, radiusVector);
    maxSpeed = Se2Controls.maxSpeed(controls);
  }

  @Override // Cost function
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  @Override // Heuristic function
  public Scalar minCostToGoal(Tensor tensor) {
    return Ramp.of(d_xy(tensor).divide(maxSpeed)); // Euclidean distance
  }
}
