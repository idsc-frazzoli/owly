// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Ramp;

/** heuristic adds max speed of available control to max norm of image gradient */
public class DeltaMinTimeGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  public static GoalInterface create(Tensor center, Scalar radius, Scalar maxMove) {
    return new DeltaMinTimeGoalManager(new SphericalRegion(center, radius), maxMove);
  }

  // ---
  private final SphericalRegion sphericalRegion;
  /** unit of maxSpeed is velocity, e.g. [m/s] */
  private final Scalar maxSpeed;

  public DeltaMinTimeGoalManager(SphericalRegion sphericalRegion, Scalar maxSpeed) {
    super(new TimeInvariantRegion(sphericalRegion));
    this.sphericalRegion = sphericalRegion;
    this.maxSpeed = maxSpeed;
  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode.stateTime(), trajectory); // unit [s]
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(sphericalRegion.apply(x).divide(maxSpeed)); // unit [m] / [m/s] simplifies to [s]
  }
}
