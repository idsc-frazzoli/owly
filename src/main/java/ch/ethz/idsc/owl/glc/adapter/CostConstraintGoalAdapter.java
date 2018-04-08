// code by jph
package ch.ethz.idsc.owl.glc.adapter;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import ch.ethz.idsc.owl.glc.core.Constraint;
import ch.ethz.idsc.owl.glc.core.GlcNode;
import ch.ethz.idsc.owl.glc.core.GoalInterface;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** combines multiple cost functions
 * 
 * @see GoalAdapter */
public class CostConstraintGoalAdapter implements GoalInterface {
  /** @param goalInterface
   * @param constraintCollection
   * @return */
  public static GoalInterface of(GoalInterface goalInterface, Collection<Constraint> constraintCollection) {
    if (constraintCollection.isEmpty())
      return goalInterface;
    return new CostConstraintGoalAdapter(goalInterface, constraintCollection);
  }

  private final Collection<Constraint> constraintCollection;
  private final GoalInterface goalInterface;

  private CostConstraintGoalAdapter(GoalInterface goalInterface, Collection<Constraint> constraintCollection) {
    this.constraintCollection = constraintCollection;
    this.goalInterface = goalInterface;
  }

  @Override
  public boolean isSatisfied(GlcNode glcNode, GlcNode prev, List<StateTime> trajectory, Flow flow) {
    /* return constraintCollection.stream() //
     * .map(constraint -> constraint.isSatisfied(glcNode, trajectory, flow)) //
     * .reduce(Boolean::logicalAnd).get(); */
    List<Constraint> listConst = (List) constraintCollection;
    return listConst.get(0).isSatisfied(glcNode, prev, trajectory, flow);
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return goalInterface.minCostToGoal(x);
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return goalInterface.costIncrement(glcNode, trajectory, flow);
  }

  @Override
  public Optional<StateTime> firstMember(List<StateTime> trajectory) {
    return goalInterface.firstMember(trajectory);
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    return goalInterface.isMember(stateTime);
  }
}
