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
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** heuristic adds max speed of available control to max norm of image gradient */
public class DeltaMinTimeGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;
  private final Scalar radius;
  /** unit of maxMove is speed, e.g. [m/s] */
  private final Scalar maxMove;

  /** @param center
   * @param radius
   * @param controls
   * @param maxNormGradient */
  public DeltaMinTimeGoalManager(Tensor center, Scalar radius, Scalar maxMove) {
    super(new TimeInvariantRegion(new SphericalRegion(center, radius)));
    this.center = center;
    this.radius = radius;
    // TODO this is/should be lipschitz constant of DeltaStateSpaceModel
    this.maxMove = maxMove;
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    // unit [s]
    return StateTimeTrajectories.timeIncrement(glcNode.stateTime(), trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // unit [m] / [m/s] simplifies to [s]
    return Ramp.of(Norm._2.between(x, center).subtract(radius).divide(maxMove));
  }
}
