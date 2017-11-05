// code by jph
package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.flow.Integrator;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** exact integration of flow using matrix exponential and logarithm.
 * states are encoded in the default coordinates of the se2 lie-algebra. */
public enum Se2Integrator implements Integrator {
  INSTANCE;
  // ---
  /** Parameter description:
   * g in SE2
   * h in R */
  @Override
  public Tensor step(Flow flow, Tensor g, Scalar h) {
    return Se2Utils.integrate(g, flow.getU().multiply(h));
  }
}
