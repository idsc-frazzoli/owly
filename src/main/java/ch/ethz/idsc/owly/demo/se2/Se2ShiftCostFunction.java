// code by jl
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owl.data.DontModify;
import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Sign;

/** cost function that penalizes the switching between forwards and backwards driving */
// DO NOT MODIFY THIS CLASS SINCE THE FUNCTIONALITY IS USED IN MANY DEMOS
@DontModify
public final class Se2ShiftCostFunction implements CostFunction {
  private final Scalar shiftPenalty;

  public Se2ShiftCostFunction(Scalar shiftPenalty) {
    this.shiftPenalty = shiftPenalty;
  }

  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    Flow ante = glcNode.flow(); // == null if glcNode is root
    return Objects.nonNull(ante) && Sign.isNegative(ante.getU().Get(0).multiply(flow.getU().Get(0))) //
        ? shiftPenalty
        : RealScalar.ZERO;
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor tensor) {
    return RealScalar.ZERO;
  }
}
