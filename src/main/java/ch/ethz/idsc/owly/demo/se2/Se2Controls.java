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
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.sca.N;

public enum Se2Controls {
  ;
  /** @param speed, positive for forward, and negative for backward
   * @param rate of steering
   * @return */
  public static Flow singleton(Scalar speed, Tensor rate) {
    return StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, //
        N.DOUBLE.of(Tensors.of(speed, RealScalar.ZERO, rate.Get().multiply(speed))));
  }

  /** @param rate_max maximum turning rate in [rad/s]
   * @param num
   * @return num equidistant turning rates in the interval [-rate_max, +rate_max] */
  public static Collection<Flow> createControls(Scalar rate_max, int num) {
    if (num % 2 == 1)
      ++num;
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(rate_max.negate(), rate_max, num))
      list.add(singleton(RealScalar.ONE, angle));
    return Collections.unmodifiableList(list);
  }

  public static Collection<Flow> createControlsForwardAndReverse(Scalar angle_max, int num) {
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(angle_max.negate(), angle_max, num)) {
      list.add(singleton(RealScalar.ONE, angle));
      list.add(singleton(RealScalar.ONE.negate(), angle));
    }
    return list;
  }

  /** @param controls
   * @return m/s */
  public static Scalar maxSpeed(Collection<Flow> controls) {
    return controls.stream().map(Flow::getU).map(u -> u.Get(0).abs()).reduce(Max::of).get();
  }

  /** @param controls
   * @return rad/s */
  public static Scalar maxTurning(Collection<Flow> controls) {
    return controls.stream().map(Flow::getU).map(u -> u.Get(2).abs()).reduce(Max::of).get();
  }
}
