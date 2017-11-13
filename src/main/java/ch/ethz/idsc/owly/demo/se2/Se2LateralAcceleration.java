// code by jl and jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.StateTimeTrajectories;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.AbsSquared;

/** Paden thesis (5.5.13) on p.57
 * cost of lateral acceleration (believed to be correlated with rider comfort)
 * 2*u^2 where d_theta(t) = u(t)
 * 
 * in the implementation below we simply use the formula u^2 */
public enum Se2LateralAcceleration implements CostFunction {
  COSTFUNCTION;
  // ---
  /** Curvature is changed angle over distance covered */
  @Override // from CostIncrementFunction
  public Scalar costIncrement(GlcNode glcNode, List<StateTime> trajectory, Flow flow) {
    Scalar dt = StateTimeTrajectories.timeIncrement(glcNode, trajectory);
    return AbsSquared.FUNCTION.apply(flow.getU().Get(2)).multiply(dt);
  }

  @Override // from HeuristicFunction
  public Scalar minCostToGoal(Tensor x) {
    return RealScalar.ZERO;
  }
}