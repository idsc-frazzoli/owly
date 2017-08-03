// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum path length
 * path length is measured in Euclidean distance */
public class RnSimpleCircleGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor center;
  private final Scalar radius;

  /** constructor creates a spherical region in R^n with given center and radius.
   * distance measure is Euclidean distance.
   * 
   * @param center vector with length == n
   * @param radius positive */
  public RnSimpleCircleGoalManager(Tensor center, Scalar radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, Array.of(l -> radius, center.length()))));
    GlobalAssert.that(Scalars.lessThan(RealScalar.ZERO, radius));
    this.center = center;
    this.radius = radius;
  }

  // TODO JONAS document what is the purpose of this constructor
  public RnSimpleCircleGoalManager(Region region, Tensor center, Scalar radius) {
    super(new TimeInvariantRegion(region));
    if (!(region instanceof EllipsoidRegion))
      throw new RuntimeException();
    this.center = center;
    this.radius = radius;
  }

  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return Norm._2.of(from.state().subtract(StateTimeTrajectories.getLast(trajectory).state()));
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // implementation is asserted by tests.
    // for modifications create a different class.
    return Ramp.of(Norm._2.of(x.subtract(center)).subtract(radius)); // <- do not change
  }
}
