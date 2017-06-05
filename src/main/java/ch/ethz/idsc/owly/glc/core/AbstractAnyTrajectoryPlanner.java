// code by jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.owly.math.state.CostFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/* package */ abstract class AbstractAnyTrajectoryPlanner extends TrajectoryPlanner {
  /* not final */ CostFunction costFunction;

  protected AbstractAnyTrajectoryPlanner(Tensor eta, CostFunction costFunction) {
    super(eta);
    this.costFunction = costFunction;
  }
  // TODO JONAS extract functions here that are common to both:
  // simpleAny and Any traj planner
  // DOCUMENT what functions do

  public abstract int switchRootToState(Tensor state);

  public abstract int switchRootToNode(GlcNode newRoot);

  @Override
  protected final GlcNode createRootNode(Tensor x) {
    return GlcNode.of(null, new StateTime(x, RealScalar.ZERO), RealScalar.ZERO, //
        costFunction.minCostToGoal(x));
  }
}
