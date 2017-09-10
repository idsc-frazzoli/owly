// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.List;

import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class MultiCostFunction implements CostFunction {
  public static CostFunction create(CostFunction... costFunctions) {
    return null;
  }
  // ---

  private final List<CostFunction> list;

  private MultiCostFunction(List<CostFunction> list) {
    this.list = list;
  }

  @Override
  public Scalar minCostToGoal(Tensor x) {
    return list.stream() //
        .map(costFunction -> costFunction.minCostToGoal(x)) //
        .reduce(Scalar::add).get();
  }

  @Override
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    return list.stream() //
        .map(costFunction -> costFunction.costIncrement(glcNode, trajectory, flow)) //
        .reduce(Scalar::add).get();
  }
}
