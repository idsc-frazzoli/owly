// code by jph and jl
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public abstract class TwdAbstractGoalManager implements Region<Tensor>, CostFunction {
  protected final Tensor center;
  protected final Scalar tolerance_xy;
  final Scalar tolerance_angle;

  /** @param center {goal_x, goal_y, goal_theta}
   * @param tolerance_xy
   * @param tolerance_angle in [rad] */
  protected TwdAbstractGoalManager(Tensor center, Scalar tolerance_xy, Scalar tolerance_angle) {
    this.center = center;
    this.tolerance_xy = tolerance_xy;
    this.tolerance_angle = tolerance_angle;
  }

  @Override // from Region
  public final boolean isMember(Tensor tensor) {
    boolean status = true;
    status &= Scalars.lessEquals(TwdHelper.errorPosition(tensor, center), tolerance_xy);
    status &= Scalars.lessEquals(TwdHelper.errorRotation(tensor, center), tolerance_angle);
    return status;
  }

  public final GoalInterface getGoalInterface() {
    return new GoalAdapter(this, SimpleTrajectoryRegionQuery.timeInvariant(this));
  }
}