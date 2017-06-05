// code by jl
package ch.ethz.idsc.owly.glc.core;

import ch.ethz.idsc.tensor.Tensor;

/* package */ abstract class AbstractAnyTrajectoryPlanner extends TrajectoryPlanner {
  protected AbstractAnyTrajectoryPlanner(Tensor eta) {
    super(eta);
  }
  // TODO extract functions here that are common to both:
  // simpleAny and Any traj planner
  // DOCUMENT what functions do

  public abstract int switchRootToState(Tensor state);

  public abstract int switchRootToNode(GlcNode newRoot);
}
