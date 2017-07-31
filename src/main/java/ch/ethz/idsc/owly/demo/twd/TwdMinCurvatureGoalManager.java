// code by jph and jl
package ch.ethz.idsc.owly.demo.twd;

import java.util.List;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Ramp;

/** Se2 goal region is not elliptic, therefore we implement {@link Region}
 * 
 * bapaden phd thesis: (6.4.10) */
public class TwdMinCurvatureGoalManager extends TwdAbstractGoalManager {
  public TwdMinCurvatureGoalManager(Tensor center, Scalar tolerance_xy, Scalar tolerance_angle) {
    super(center, tolerance_xy, tolerance_angle);
  }

  @Override // Cost Function
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    StateTime end = trajectory.get(trajectory.size() - 1);
    // J(x,u) = 1+delta(theta)²
    return RealScalar.ONE.add(Power.of(end.state().Get(2).subtract(from.state().Get(2)), 2));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    Tensor xy = x.extract(0, 2);
    Scalar dxy = Norm._2.of(xy.subtract(center.extract(0, 2))).subtract(tolerance_xy);
    return Ramp.of(dxy);
  }
}