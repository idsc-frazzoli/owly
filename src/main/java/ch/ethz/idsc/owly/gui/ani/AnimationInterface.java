// code by jph
package ch.ethz.idsc.owly.gui.ani;

import ch.ethz.idsc.tensor.Scalar;

public interface AnimationInterface {
  void integrate(Scalar now);
}
