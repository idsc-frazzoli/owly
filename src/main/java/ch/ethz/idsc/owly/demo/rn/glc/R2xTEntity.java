// code by jph
package ch.ethz.idsc.owly.demo.rn.glc;

import java.util.Collection;

import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/** 
 * 
 */
/* package */ class R2xTEntity extends R2Entity {
  public R2xTEntity(Tensor state) {
    super(state);
    represent_entity = StateTime::joined;
  }

  // TODO not sure what is a good approach here:
  private static final Tensor WEIGHT = Tensors.vector(1.0, 1.0, 0.2);

  @Override
  protected Scalar distance(Tensor x, Tensor y) {
    Tensor d = x.subtract(y);
    return d.pmul(WEIGHT).dot(d).Get();
  }

  @Override
  public Scalar delayHint() {
    return RealScalar.of(1.0);
  }

  @Override
  public TrajectoryPlanner createTrajectoryPlanner(TrajectoryRegionQuery obstacleQuery, Tensor goal) {
    TrajectoryPlanner trajectoryPlanner = super.createTrajectoryPlanner(obstacleQuery, goal);
    trajectoryPlanner.represent = StateTime::joined;
    return trajectoryPlanner;
  }

  @Override
  protected Tensor eta() {
    return Tensors.vector(8, 8, 4);
  }

  @Override
  Collection<Flow> createControls() {
    Collection<Flow> collection = super.createControls();
    // collection.add(R2Controls.stayPut(2)); // <- does not go well with min-dist cost function
    return collection;
  }
}
