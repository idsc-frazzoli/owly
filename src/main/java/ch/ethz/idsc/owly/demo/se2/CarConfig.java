// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import ch.ethz.idsc.owly.math.StateSpaceModels;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.sca.N;

public class CarConfig {
  private final Scalar speed;
  private final Scalar rate_max;

  /** @param speed with unit [m*s^-1]
   * @param rate_max with unit [rad*m^-1], i.e. the amount of
   * rotation [rad] performed per distance [m^-1] */
  public CarConfig(Scalar speed, Scalar rate_max) {
    this.speed = speed;
    this.rate_max = rate_max;
  }

  /** @param speed, positive for forward, and negative for backward, unit [m/s]
   * @param rate of steering, unit [rad/m]
   * @return flow with u == {speed[m*s^-1], 0.0, (rate*speed)[rad*s^-1]} */
  /* package for testing */ static Flow singleton(Scalar speed, Tensor rate) {
    return StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, //
        N.DOUBLE.of(Tensors.of(speed, RealScalar.ZERO, rate.multiply(speed))));
  }

  /** @param rate_max maximum turning rate in [rad/m]
   * @param num
   * @return num equidistant turning rates in the interval [-rate_max, +rate_max] */
  public Collection<Flow> createControls(int num) {
    if (num % 2 == 1)
      ++num;
    List<Flow> list = new ArrayList<>();
    for (Tensor rate : Subdivide.of(rate_max.negate(), rate_max, num))
      list.add(singleton(speed, rate));
    return Collections.unmodifiableList(list);
  }

  public Collection<Flow> createControlsForwardAndReverse(int num) {
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(rate_max.negate(), rate_max, num)) {
      list.add(singleton(speed, angle));
      list.add(singleton(speed.negate(), angle));
    }
    return list;
  }
}
