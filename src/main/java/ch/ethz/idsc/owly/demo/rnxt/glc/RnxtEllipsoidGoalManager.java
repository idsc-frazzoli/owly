// code by jph & jl
package ch.ethz.idsc.owly.demo.rnxt.glc;

import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.NoHeuristic;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;

/** objective is minimum path length
 * path length is measured in Euclidean distance */
public class RnxtEllipsoidGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface, NoHeuristic {
  // protected when used in subclasses
  // TODO JONAS these are not used anywhere in the formulas
  private final Tensor center;
  private final Tensor radius;

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
    for (Tensor radiusTemp : radius)
      GlobalAssert.that(Scalars.lessThan(RealScalar.ZERO, (Scalar) radiusTemp));
    this.radius = radius;
    this.center = center;
  }

  /** shortest Time Cost */
  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }
}
