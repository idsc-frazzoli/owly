// code by jph and jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

/** Se2 goal region is not elliptic, therefore we implement {@link Region}
 * 
 * bapaden phd thesis: (6.4.10) */
public class Se2DefaultGoalManager implements Region, CostFunction {
  static final Mod PRINCIPAL = Mod.function(RealScalar.of(2 * Math.PI), RealScalar.of(-Math.PI));
  // ---
  final Tensor center;
  final Tensor radiusVector; // TODO outside access to this member should not happen

  public Se2DefaultGoalManager(Tensor center, Tensor radiusVector) {
    this.center = center;
    this.radiusVector = radiusVector;
  }

  @Override // Cost Function
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    // integrate(1,t)
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  @Override // Heuristic function
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }

  @Override
  public boolean hasHeuristic() {
    return false;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    Tensor cur_xy = tensor.extract(0, 2);
    Scalar cur_angle = tensor.Get(2);
    boolean status = true;
    status &= Scalars.lessEquals(Norm._2.of(cur_xy.subtract(center.extract(0, 2))), radiusVector.Get(1));
    status &= Scalars.lessEquals(PRINCIPAL.apply(cur_angle.subtract(center.Get(2))).abs(), radiusVector.Get(2));
    return status;
  }

  public GoalInterface getGoalInterface() {
    return new GoalAdapter(this, new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this)));
  }
}