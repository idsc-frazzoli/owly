// code by jph & jl
package ch.ethz.idsc.owly.demo.rnxt.glc;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.glc.core.NoHeuristic;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TimeInvariantRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Ramp;
import ch.ethz.idsc.tensor.sca.Sqrt;

/** objective is minimum path length
 * path length is measured in Euclidean distance */
// TODO can implementation be generic: RnxT ?
public class R2xtHeuristicEllipsoidGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface, NoHeuristic {
  // protected when used in subclasses
  private final Tensor center;
  private final Tensor radius;

  /** constructor creates a spherical region in R^n x T with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cyclinder
   * 
   * @param center vector with length == n
   * @param radius positive */
  public R2xtHeuristicEllipsoidGoalManager(Tensor center, Scalar radius) {
    this(center, Array.of(l -> radius, center.length()));
  }

  /** constructor creates a ellipsoid region in R^n x T with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cyclinder
   * 
   * @param center vector with length == n
   * @param radius vector with length == n & positive in all entrys */
  public R2xtHeuristicEllipsoidGoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    if (center.length() != 3)// needs to be R2 x T therefore 3 states
      throw new RuntimeException();
    // TODO 3rd component of center/radius is never used -> misleading
    // ... if this is the final design then drop last entry before passing to constructor
    this.radius = radius;
    this.center = center;
  }

  /** shortest Time Cost */
  @Override
  public Scalar costIncrement(StateTime from, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(from, trajectory);
  }

  /** Ellipsoid with axis: a,b and vector from Center: v = (x,y)
   * specific radius at intersection:
   * r :
   * 
   * a*b * ||v||
   * ---------------
   * sqrt(a²y² + b²x²) */
  @Override
  public Scalar minCostToGoal(Tensor x) {
    int toIndex = x.length() - 1;
    Tensor r2State = x.extract(0, toIndex);
    Tensor r2Center = center.extract(0, toIndex);
    Tensor r2Vector = r2State.subtract(r2Center);
    Tensor r2Radius = radius.extract(0, toIndex);
    // TODO JONAS state origin of formulas for verification purpose
    // TODO JONAS use Hypot.BIFUNCTION.apply(a, b) instead of sqrt(a^2+b^2)
    Scalar root = Sqrt.of( //
        Power.of(r2Radius.Get(0).multiply(r2Vector.Get(1)), 2).add( //
            Power.of(r2Radius.Get(1).multiply(r2Vector.Get(0)), 2)));
    // ---
    Scalar specificRadius = radius.Get(0).multiply(radius.Get(1)).multiply(Norm._2.of(r2State.subtract(r2Center))).divide(root);
    return Ramp.of(Norm._2.of(r2State.subtract(r2Center)).subtract(specificRadius)); // <- do not change
  }
}
