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
import ch.ethz.idsc.tensor.red.Norm2Squared;
import ch.ethz.idsc.tensor.sca.Ramp;

/** Nonholonomic Wheeled Robot
 * 
 * bapaden phd thesis: (5.5.13) */
public final class Se2MinDistCurvGoalManager extends Se2AbstractGoalManager {
  public Se2MinDistCurvGoalManager(Tensor goal, Tensor radiusVector) {
    super(goal, radiusVector);
  }

  @Override // Cost function
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    // Cost increases with time and input length
    // TODO currently all Se2models only change angle, no amplitude changes
    // integrate(||u||²+1,t)
    return RealScalar.ONE.add(Norm2Squared.ofVector(flow.getU()))//
        .multiply(StateTimeTrajectories.timeIncrement(glcNode, trajectory));
  }

  @Override // Heuristic function
  public Scalar minCostToGoal(Tensor tensor) {
    return Ramp.of(d_xy(tensor).subtract(radiusSpace())); // Euclidean distance
  }
}
