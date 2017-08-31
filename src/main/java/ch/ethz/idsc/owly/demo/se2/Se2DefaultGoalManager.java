// code by jph and jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
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
  static final Mod PRINCIPAL = Mod.function(2 * Math.PI, -Math.PI);
  // ---
  protected final Tensor center;
  protected final Tensor radiusVector;

  /** @param center
   * @param radiusVector with 3 entries the first 2 of which have to be identical */
  public Se2DefaultGoalManager(Tensor center, Tensor radiusVector) {
    GlobalAssert.that(radiusVector.get(0).equals(radiusVector.get(1)));
    this.center = center.unmodifiable();
    this.radiusVector = radiusVector.unmodifiable();
  }

  protected Scalar radiusSpace() {
    return radiusVector.Get(0);
  }

  protected Scalar radiusAngle() {
    return radiusVector.Get(2);
  }

  @Override // from CostFunction
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    StateTime from = node.stateTime();
    // integrate(1,t)
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  @Override // from CostFunction
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }

  @Override // from Region
  public boolean isMember(Tensor tensor) {
    Tensor cur_xy = tensor.extract(0, 2);
    Scalar cur_angle = tensor.Get(2);
    boolean status = true;
    status &= Scalars.lessEquals(Norm._2.ofVector(cur_xy.subtract(center.extract(0, 2))), radiusSpace());
    status &= Scalars.lessEquals(PRINCIPAL.apply(cur_angle.subtract(center.Get(2))).abs(), radiusAngle());
    return status;
  }

  public GoalInterface getGoalInterface() {
    return new GoalAdapter(this, new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this)));
  }
}