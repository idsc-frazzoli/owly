// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.util.Collection;
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
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** heuristic adds max speed of available control to max norm of image gradient */
public class DeltaMinTimeGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;
  private final Scalar radius;
  private final Scalar maxMove;

  /** @param center
   * @param radius
   * @param controls
   * @param maxNormGradient */
  public DeltaMinTimeGoalManager( //
      Tensor center, Scalar radius, Collection<Flow> controls, Scalar maxNormGradient) {
    super(new TimeInvariantRegion(new SphericalRegion(center, radius)));
    this.center = center;
    this.radius = radius;
    maxMove = DeltaControls.maxSpeed(controls).add(maxNormGradient);
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode.stateTime(), trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return Ramp.of(Norm._2.ofVector(x.subtract(center)).subtract(radius).divide(maxMove));
  }
}