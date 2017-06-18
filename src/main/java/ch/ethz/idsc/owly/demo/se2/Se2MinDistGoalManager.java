// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** Nonholonomic Wheeled Robot
 * 
 * bapaden phd thesis: (5.5.13) */
public class Se2MinDistGoalManager extends Se2DefaultGoalManagerExt {
  public Se2MinDistGoalManager(Tensor xy, Scalar angle, Scalar radius, Scalar angle_delta) {
    super(xy, angle, radius, angle_delta);
  }

  @Override // Cost function
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    // Cost increases with time and input length
    // TODO currently all Se2models only change angle, no amplitude changes
    // integrate(||u||²+1,t)
    return RealScalar.ONE.add(Norm._2Squared.of(flow.getU()))//
        .multiply(Trajectories.timeIncrement(from, trajectory));
  }

  @Override // Heuristic function
  public Scalar minCostToGoal(Tensor x) {
    Tensor cur_xy = x.extract(0, 2);
    // Euclidean distance
    Scalar dxy = Norm._2.of(cur_xy.subtract(xy)).subtract(radius);
    return Ramp.of(dxy);
  }
}
