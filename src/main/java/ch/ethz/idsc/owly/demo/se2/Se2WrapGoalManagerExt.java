// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.GoalAdapter;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.sca.Ramp;

/** minimizes driving time (=distance, since unit speed)
 * 
 * {@link Se2WrapGoalManagerExt} works with {@link Se2Wrap} as well as with {@link TnIdentityWrap} */
public class Se2WrapGoalManagerExt implements Region, CostFunction {
  private final CoordinateWrap coordinateWrap;
  private final Se2DefaultGoalManagerExt goalManager;

  /** @param coordinateWrap */
  public Se2WrapGoalManagerExt(CoordinateWrap coordinateWrap, Se2DefaultGoalManagerExt goalManager) {
    this.coordinateWrap = coordinateWrap;
    this.goalManager = goalManager;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return goalManager.costIncrement(from, trajectory, flow);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(coordinateWrap.distance(x, goalManager.center).subtract(goalManager.radiusVector.Get(1)));
  }

  @Override
  public boolean isMember(Tensor x) {
    return Scalars.isZero(Ramp.of(coordinateWrap.distance(x, goalManager.center)//
        .subtract(coordinateWrap.distance(Tensors.vector(0, 0, 0), goalManager.radiusVector))));
  }

  public GoalInterface getGoalInterface() {
    return new GoalAdapter(this, new SimpleTrajectoryRegionQuery(new TimeInvariantRegion(this)));
  }
}