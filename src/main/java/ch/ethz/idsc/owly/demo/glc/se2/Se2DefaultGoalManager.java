// code by jph and jl
package ch.ethz.idsc.owly.demo.glc.se2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

/** Se2 goal region is not elliptic, therefore we implement {@link Region}
 * 
 * bapaden phd thesis: (6.4.10) */
public class Se2DefaultGoalManager implements Region, CostFunction {
  static final Mod PRINCIPAL = Mod.function(RealScalar.of(2 * Math.PI), RealScalar.of(-Math.PI));
  // ---
  final Tensor xy;
  final Scalar angle;
  final Tensor center;
  final Scalar radius;
  final Scalar angle_delta;
  final Tensor radiusVector;

  public Se2DefaultGoalManager(Tensor xy, Scalar angle, Scalar radius, Scalar angle_delta) {
    this.xy = xy;
    this.angle = angle;
    this.center = Tensors.of(xy.Get(0), xy.Get(1), angle);
    this.radius = radius;
    this.angle_delta = angle_delta;
    this.radiusVector = Tensors.of(radius, radius, angle_delta);
  }

  @Override // Cost Function
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    // integrate(1,t)
    return Trajectories.timeIncrement(from, trajectory);
  }

  @Override // Heuristic function
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    Tensor cur_xy = tensor.extract(0, 2);
    Scalar cur_angle = tensor.Get(2);
    boolean status = true;
    status &= Scalars.lessEquals(Norm._2.of(cur_xy.subtract(xy)), radius);
    status &= Scalars.lessEquals(PRINCIPAL.apply(cur_angle.subtract(angle)).abs(), angle_delta);
    return status;
  }

  public GoalInterface getGoalInterface() {
    return new GoalAdapter(this, new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this)));
  }
}