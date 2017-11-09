// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.StateTimeTensorFunction;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** several magic constants are hard-coded in the implementation.
 * that means, the functionality does not apply to all examples universally. */
class Se2xTEntity extends Se2Entity implements StateTimeTensorFunction {
  Se2xTEntity(Tensor state) {
    super(state); // initial position
    represent_entity = StateTime::joined;
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x.extract(0, 3), y.extract(0, 3));
    // throw TensorRuntimeException.of(x, y);
  }

  @Override
  public Scalar delayHint() {
    return RealScalar.of(2.0);
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    TrajectoryPlanner trajectoryPlanner = super.createTrajectoryPlanner(obstacleQuery, goal);
    // trajectoryPlanner.represent = StateTimeTensorFunction.state(SE2WRAP::represent);
    trajectoryPlanner.represent = this::apply;
    return trajectoryPlanner;
  }

  @Override
  public Tensor apply(StateTime stateTime) {
    return SE2WRAP.represent(stateTime.state()).copy().append(stateTime.time());
  }

  @Override
  protected Tensor eta() {
    return super.eta().copy().append(RealScalar.of(4));
  }
}