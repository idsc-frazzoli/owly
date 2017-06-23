// code by jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** Bulirsch Stoer method
 * 
 * Numerical Recipes 3rd Edition Section 17.3.2 */
enum BulirschStoerIntegrator implements Integrator {
  INSTANCE;
  // ---
  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    // TODO implement
    throw new RuntimeException();
  }
}
