// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.StateTimeCoordinateWrap;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** several magic constants are hard-coded in the implementation.
 * that means, the functionality does not apply to all examples universally. */
class CarxTEntity extends CarEntity {
  CarxTEntity(StateTime stateTime) {
    super(stateTime); // initial position
    represent_entity = StateTime::joined;
  }

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    return SE2WRAP.distance(x.extract(0, 3), y.extract(0, 3));
  }

  @Override
  public Scalar delayHint() {
    return RealScalar.of(2.0);
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    TrajectoryPlanner trajectoryPlanner = super.createTrajectoryPlanner(obstacleQuery, goal);
    trajectoryPlanner.represent = new StateTimeCoordinateWrap(SE2WRAP);
    return trajectoryPlanner;
  }

  @Override
  protected Tensor eta() {
    return super.eta().copy().append(RealScalar.of(4));
  }
}
