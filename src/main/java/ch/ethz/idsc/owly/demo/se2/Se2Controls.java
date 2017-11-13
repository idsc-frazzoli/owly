// code by jph
package ch.ethz.idsc.owly.demo.se2;

import java.util.Collection;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.red.Max;

public enum Se2Controls {
  ;
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
