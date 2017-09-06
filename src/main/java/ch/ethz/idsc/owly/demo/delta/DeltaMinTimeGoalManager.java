// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** default goal manager for delta example that does not make use of max norm of flow */
public class DeltaMinTimeGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;
  private final Scalar radius;
  private final Scalar maxSpeed;

  public DeltaMinTimeGoalManager(Tensor center, Scalar radius, Collection<Flow> controls) {
    super(new TimeInvariantRegion(EllipsoidRegion.spherical(center, radius)));
    this.center = center;
    this.radius = radius;
    maxSpeed = DeltaControls.maxSpeed(controls);
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode.stateTime(), trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(Norm._2.ofVector(x.subtract(center)).subtract(radius).divide(maxSpeed));
  }
}
