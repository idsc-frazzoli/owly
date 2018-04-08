// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Subdivide;

/** controls allow to drive forward and backward */
public class CarVelocityFlows extends CarFlows {
  private final Tensor velocities;
  private final Scalar rate_max;

  /** @param velocity with unit [m*s^-1]
   * @param velocity resolution
   * @param rate_max with unit [rad*m^-1], i.e. the amount of
   * rotation [rad] performed per distance [m^-1] */
  public CarVelocityFlows(Scalar velocity, int velResolution, Scalar rate_max) {
    this.rate_max = rate_max;
    this.velocities = Subdivide.of(velocity.negate(), velocity, velResolution);
  }

  public CarVelocityFlows(Tensor velocities, Scalar rate_max) {
    this.rate_max = rate_max;
    this.velocities = velocities;
  }

  @Override
  public Collection<Flow> getFlows(int resolution) {
    List<Flow> list = new ArrayList<>();
    for (Tensor sp : velocities) {
      for (Tensor angle : Subdivide.of(rate_max.negate(), rate_max, resolution)) {
        list.add(singleton((Scalar) sp, angle));
      }
    }
    return list;
  }
}