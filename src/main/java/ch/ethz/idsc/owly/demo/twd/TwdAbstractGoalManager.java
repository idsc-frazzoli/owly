// code by jph and jl
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

public abstract class TwdAbstractGoalManager implements Region, CostFunction {
  static final Mod PRINCIPAL = Mod.function(RealScalar.of(2 * Math.PI), RealScalar.of(-Math.PI));
  // ---
  final Tensor center;
  final Scalar tolerance_xy;
  final Scalar tolerance_angle;

  /** @param center
   * @param tolerance_xy
   * @param tolerance_angle in [rad] */
  protected TwdAbstractGoalManager(Tensor center, Scalar tolerance_xy, Scalar tolerance_angle) {
    this.center = center;
    this.tolerance_xy = tolerance_xy;
    this.tolerance_angle = tolerance_angle;
  }

  @Override
  public final boolean isMember(Tensor tensor) {
    Tensor xy = tensor.extract(0, 2);
    Scalar angle = tensor.Get(2);
    boolean status = true;
    status &= Scalars.lessEquals(Norm._2.of(xy.subtract(center.extract(0, 2))), tolerance_xy);
    status &= Scalars.lessEquals(PRINCIPAL.apply(angle.subtract(center.Get(2))), tolerance_angle);
    return status;
  }

  public final GoalInterface getGoalInterface() {
    return new GoalAdapter(this, new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this)));
  }
}