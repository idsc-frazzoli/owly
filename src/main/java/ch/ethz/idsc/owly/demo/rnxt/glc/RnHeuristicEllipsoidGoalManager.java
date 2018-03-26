// code by jl
package ch.ethz.idsc.owly.demo.rnxt.glc;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owl.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owl.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.region.EllipsoidRegion;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TimeInvariantRegion;
import ch.ethz.idsc.owly.demo.rn.RnControls;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.red.Hypot;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum time
 * path length is measured in Euclidean distance
 * Heuristic is minimum Time along Euclidean distance */
@Deprecated
/* package */ class RnHeuristicEllipsoidGoalManager extends SimpleTrajectoryRegionQuery implements GoalInterface {
  /** constructor creates a spherical region in R^n with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cylinder
   * 
   * @param center vector with length == n
   * @param radius positive */
  public static GoalInterface create(Tensor center, Scalar radius, Collection<Flow> controls) {
    return new RnHeuristicEllipsoidGoalManager(new EllipsoidRegion( //
        center, Array.of(l -> radius, center.length())), controls);
  }

  // ---
  private final EllipsoidRegion ellipsoidRegion;
  private final Scalar maxSpeed;

  /** constructor creates a ellipsoid region in R^n x T with given center and radius.
   * distance measure is Euclidean distance, if radius(i) = infinity => cylinder
   * 
   * @param center vector with length == n
   * @param radius vector with length == n & positive in all entries */
  public RnHeuristicEllipsoidGoalManager(EllipsoidRegion ellipsoidRegion, Collection<Flow> controls) {
    super(new TimeInvariantRegion(ellipsoidRegion));
    this.ellipsoidRegion = ellipsoidRegion;
    maxSpeed = RnControls.maxSpeed(controls);
  }

  /** shortest Time Cost */
  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return StateTimeTrajectories.timeIncrement(glcNode, trajectory);
  }

  /** Ellipsoid with axis: a,b and vector from Center: v = (x,y)
   * reference (not relevant for problem):
   * https://math.stackexchange.com/questions/432902/how-to-get-the-radius-of-an-ellipse-at-a-specific-angle-by-knowing-its-semi-majo
   * specific radius at intersection r =
   * 
   * a*b * ||v||
   * ---------------
   * sqrt(a²y² + b²x²) */
  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor x) {
    // FIXME the formula is conceptually wrong:
    // 1) function may return larger values than necessary
    // 2) SOLVED: cost increment has unit time, while minCost has length unit
    Tensor rnVector = x.subtract(ellipsoidRegion.center());
    Scalar root = Hypot.BIFUNCTION.apply(ellipsoidRegion.radius().Get(0).multiply(rnVector.Get(1)), ellipsoidRegion.radius().Get(1).multiply(rnVector.Get(0)));
    // ---
    Scalar specificRadius = ellipsoidRegion.radius().Get(0).multiply(ellipsoidRegion.radius().Get(1)).multiply(Norm._2.between(x, ellipsoidRegion.center()))
        .divide(root);
    // shortest distance/speed = time towards goal, minus radius of (elliptic) goalregion,
    // as we need to guarantee that minCostToGoal(x in Goal) == 0;
    // TODO implement formula: https://math.stackexchange.com/questions/90974/calculating-distance-of-a-point-from-an-ellipse-border
    // needs to solve a quartic equation each time
    return Ramp.of(Norm._2.between(x, ellipsoidRegion.center()).subtract(specificRadius)).divide(maxSpeed); // <- do not change
  }
}
