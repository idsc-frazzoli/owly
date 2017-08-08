// code by jph & jl
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Ramp;

/** objective is minimum path length
 * path length is measured in Euclidean distance */
public class RnSimpleCircleHeuristicGoalManager extends RnSimpleCircleGoalManager implements GoalInterface {
  /** constructor creates a spherical region in R^n with given center and radius.
   * distance measure is Euclidean distance.
   * 
   * @param center vector with length == n
   * @param radius positive */
  public RnSimpleCircleHeuristicGoalManager(Tensor center, Scalar radius) {
    super(center, radius);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    // implementation is asserted by tests.
    // for modifications create a different class.
    return Ramp.of(Norm._2.of(x.subtract(center)).subtract(radius)); // <- do not change
  }
}