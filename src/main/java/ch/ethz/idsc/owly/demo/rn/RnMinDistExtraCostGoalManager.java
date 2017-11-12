// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GoalInterface;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.region.SphericalRegion;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** superimpose cost from image onto normal cost */
// TODO use MultiCostFunction
public class RnMinDistExtraCostGoalManager extends RnMinDistSphericalGoalManager implements GoalInterface {
  private final CostFunction costFunction;

  public RnMinDistExtraCostGoalManager(Tensor center, Scalar radius, CostFunction costFunction) {
    super(new SphericalRegion(center, radius));
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
