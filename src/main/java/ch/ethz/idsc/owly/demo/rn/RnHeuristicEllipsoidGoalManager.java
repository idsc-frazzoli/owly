// code by jph and jl
package ch.ethz.idsc.owly.demo.rn;

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
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum time
 * path length is measured in Euclidean distance
 * Heuristic is minimum Time along Euclidean distance */
public class RnHeuristicEllipsoidGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  private final Tensor rnCenter;
  private final Tensor rnRadius;

  /** constructor creates a spherical region in R^n with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cylinder
   * 
   * @param center vector with length == n
   * @param radius positive */
  public RnHeuristicEllipsoidGoalManager(Tensor center, Scalar radius) {
    this(center, Array.of(l -> radius, center.length()));
  }

  /** constructor creates a ellipsoid region in R^n x T with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cylinder
   * 
   * @param center vector with length == n
   * @param radius vector with length == n & positive in all entrys */
  public RnHeuristicEllipsoidGoalManager(Tensor center, Tensor radius) {
    super(new TimeInvariantRegion(new EllipsoidRegion(center, radius)));
    int toIndex = center.length();
    this.rnRadius = radius.extract(0, toIndex);
    this.rnCenter = center.extract(0, toIndex);
  }

  /** shortest Time Cost */
  @Override
  public Scalar costIncrement(GlcNode node, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(node.stateTime(), trajectory);
  }

  /** Ellipsoid with axis: a,b and vector from Center: v = (x,y)
   * reference:
   * https://math.stackexchange.com/questions/432902/how-to-get-the-radius-of-an-ellipse-at-a-specific-angle-by-knowing-its-semi-majo
   * specific radius at intersection r =
   * 
   * a*b * ||v||
   * ---------------
   * sqrt(a²y² + b²x²) */
  @Override
  public Scalar minCostToGoal(Tensor x) {
    // FIXME the formula is probably conceptually wrong:
    // we don't need distance in along a certain direction but overall shortest distance regardless of direction
    Tensor rnVector = x.subtract(rnCenter);
    Scalar root = Hypot.BIFUNCTION.apply(rnRadius.Get(0).multiply(rnVector.Get(1)), rnRadius.Get(1).multiply(rnVector.Get(0)));
    // ---
    Scalar specificRadius = rnRadius.Get(0).multiply(rnRadius.Get(1)).multiply(Norm._2.between(x, rnCenter)).divide(root);
    return Ramp.of(Norm._2.between(x, rnCenter).subtract(specificRadius)); // <- do not change
  }
}
