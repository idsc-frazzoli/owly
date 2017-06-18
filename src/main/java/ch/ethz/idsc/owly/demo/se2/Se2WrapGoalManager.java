// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.demo.glc.tn.IdentityWrap;
import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.math.state.Trajectories;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Ramp;

/** minimizes driving time (=distance, since unit speed)
 * 
 * {@link Se2WrapGoalManager} works with {@link Se2Wrap} as well as with {@link IdentityWrap} */
public class Se2WrapGoalManager implements Region, CostFunction {
  private final CoordinateWrap coordinateWrap;
  private final Tensor center;
  private final Scalar radius;

  /** @param coordinateWrap
   * @param center consists of x,y,theta
   * @param radius */
  public Se2WrapGoalManager(CoordinateWrap coordinateWrap, Tensor center, Scalar radius) {
    this.coordinateWrap = coordinateWrap;
    this.center = center;
    this.radius = radius;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Trajectories.timeIncrement(from, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(coordinateWrap.distance(x, center).subtract(radius));
  }

  @Override
  public boolean isMember(Tensor x) {
    return Scalars.isZero(minCostToGoal(x));
  }

  public GoalInterface getGoalInterface() {
    return new GoalAdapter(this, new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this)));
  }
}