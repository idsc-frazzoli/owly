// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import ch.ethz.idsc.owl.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Subdivide;

/** controls only permit to drive forward */
public class CarForwardFlows extends CarFlows {
  private final Scalar speed;
  private final Scalar rate_max;

  /** @param speed with unit [m*s^-1]
   * @param rate_max with unit [rad*m^-1], i.e. the amount of
   * rotation [rad] performed per distance [m^-1] */
  public CarForwardFlows(Scalar speed, Scalar rate_max) {
    this.speed = speed;
    this.rate_max = rate_max;
  }

  @Override
  public Collection<Flow> getFlows(int resolution) {
    if (resolution % 2 == 1)
      ++resolution;
    List<Flow> list = new ArrayList<>();
    for (Tensor rate : Subdivide.of(rate_max.negate(), rate_max, resolution))
      list.add(singleton(speed, rate));
    return Collections.unmodifiableList(list);
  }
}
