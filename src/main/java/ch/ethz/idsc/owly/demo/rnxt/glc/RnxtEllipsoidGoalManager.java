// code by jph & jl
package ch.ethz.idsc.owly.demo.rnxt.glc;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum path length
 * path length is measured in Euclidean distance */
@Deprecated // TODO class is not used, candidate for deletion
class RnxtEllipsoidGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  // protected when used in subclasses
  /** constructor creates a spherical region in R^n x T with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cylinder
   * 
   * @param center vector with length == n
   * @param radius positive */
  public RnxtEllipsoidGoalManager(Tensor center, Scalar radius) {
    this(center, Array.of(l -> radius, center.length()));
  }

  /** constructor creates a ellipsoid region in R^n x T with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cylinder
   * 
   * @param center vector with length == n
   * @param radius vector with length == n & positive in all entries */
  public RnxtEllipsoidGoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    if (!center.Get(center.length() - 1).equals(Ramp.of(center.Get(center.length() - 1)))) // assert that time in center is non-negative
      throw TensorRuntimeException.of(radius);
  }

  /** shortest Time Cost */
  @Override
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    StateTime from = node.stateTime();
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }
}
