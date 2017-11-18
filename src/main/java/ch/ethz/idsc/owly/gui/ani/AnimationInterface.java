// code by jph
package ch.ethz.idsc.owly.gui.ani;

import ch.ethz.idsc.tensor.Scalar;

public interface AnimationInterface {
  /** @param now measure of real time */
  void integrate(Scalar now);
}
