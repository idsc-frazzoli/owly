// code by jph
package ch.ethz.idsc.owly.demo.delta.glc;

import ch.ethz.idsc.owly.demo.delta.ImageGradient;
import ch.ethz.idsc.owly.glc.core.StandardTrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** class controls delta using {@link StandardTrajectoryPlanner} */
/* package */ class DeltaxTEntity extends DeltaEntity {
  public DeltaxTEntity(ImageGradient imageGradient, Tensor state) {
    super(imageGradient, state);
    represent_entity = StateTime::joined;
  }

  private static final Tensor WEIGHT = Tensors.vector(1.0, 1.0, 0.2);

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    Tensor d = x.subtract(y);
    return d.pmul(WEIGHT).dot(d).Get();
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    TrajectoryPlanner trajectoryPlanner = super.createTrajectoryPlanner(obstacleQuery, goal);
    trajectoryPlanner.represent = StateTime::joined;
    return trajectoryPlanner;
  }

  @Override
  protected Tensor eta() {
    return Tensors.vector(5, 5, 4);
  }
}
