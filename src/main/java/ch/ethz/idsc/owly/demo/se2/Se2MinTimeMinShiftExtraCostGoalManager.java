// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** superimpose cost from image onto normal cost */
// TODO JAN code is redundant to R2..., use MultiCostFunction etc.
public class Se2MinTimeMinShiftExtraCostGoalManager extends Se2MinTimeMinShiftGoalManager {
  public static GoalInterface create(Tensor goal, Tensor radiusVector, Collection<Flow> controls, Scalar shiftPenalty, CostFunction costFunction) {
    return new Se2MinTimeMinShiftExtraCostGoalManager(goal, radiusVector, controls, shiftPenalty, costFunction).getGoalInterface();
  }

  // ---
  private final CostFunction costFunction;

  public Se2MinTimeMinShiftExtraCostGoalManager(Tensor goal, Tensor radiusVector, Collection<Flow> controls, Scalar shiftPenalty, CostFunction costFunction) {
    super(goal, radiusVector, controls, shiftPenalty);
    this.costFunction = costFunction;
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return super.minCostToGoal(x).add(costFunction.minCostToGoal(x));
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    Scalar impose = costFunction.costIncrement(glcNode, trajectory, flow);
    return super.costIncrement(glcNode, trajectory, flow).add(impose);
  }
}
