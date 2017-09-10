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
import ch.ethz.idsc.tensor.red.Norm;

public enum Se2Controls {
  ;
  /** @param angle of steering
   * @param speed, positive for forward, and negative for backward
   * @return */
  public static Flow singleton(Scalar angle, Scalar speed) {
    return StateSpaceModels.createFlow(Se2StateSpaceModel.INSTANCE, Tensors.of(angle, speed));
  }

  /** @param rate_max maximum turning rate in [rad/s]
   * @param num
   * @return num equidistant turning rates in the interval [-rate_max, +rate_max] */
  public static Collection<Flow> createControls(Scalar rate_max, int num) {
    if (num % 2 == 1)
      ++num;
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(rate_max.negate(), rate_max, num))
      list.add(singleton((Scalar) angle, RealScalar.ONE));
    return Collections.unmodifiableList(list);
  }

  public static Collection<Flow> createControlsForwardAndReverse(Scalar angle_max, int num) {
    List<Flow> list = new ArrayList<>();
    for (Tensor angle : Subdivide.of(angle_max.negate(), angle_max, num)) {
      list.add(singleton(angle.Get(), RealScalar.ONE));
      list.add(singleton(angle.Get(), RealScalar.ONE.negate()));
    }
    return list;
  }

  /** @param controls
   * @return m/s */
  public static Scalar maxSpeed(Collection<Flow> controls) {
    return controls.stream().map(Flow::getU).map(u -> u.Get(1).abs()).reduce(Max::of).get();
  }

  /** @param controls
   * @return rad/s */
  public static Scalar maxTurning(Collection<Flow> controls) {
    return Norm.INFINITY.ofVector(Tensor.of(controls.stream().map(Flow::getU).map(u -> u.Get(0)))) //
        .multiply(maxSpeed(controls));
  }
}
