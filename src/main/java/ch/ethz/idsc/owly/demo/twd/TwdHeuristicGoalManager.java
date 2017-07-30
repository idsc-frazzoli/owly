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
public class TwdHeuristicGoalManager extends TwdDefaultGoalManager {
  public TwdHeuristicGoalManager(Tensor center, Tensor radiusVector) {
    super(center, radiusVector);
  }

  @Override // Cost Function
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    StateTime end = trajectory.get(trajectory.size() - 1);
    // J(x,u) = 1+delta(theta)²
    return RealScalar.ONE.add(Power.of(end.state().Get(2).subtract(from.state().Get(2)), 2));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    Tensor cur_xy = x.extract(0, 2);
    Scalar cur_angle = x.Get(2);
    Scalar dxy = Norm._2.of(cur_xy.subtract(center.extract(0, 2))).subtract(radiusVector.Get(1));
    return Ramp.of(dxy);
  }
}