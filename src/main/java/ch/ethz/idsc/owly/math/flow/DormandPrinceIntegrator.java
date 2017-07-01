// code by jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** fifth-order Dormand-Prince method
 * 
 * Numerical Recipes 3rd Edition Section 17.2.3 */
enum DormandPrinceIntegrator implements Integrator {
  INSTANCE;
  // ---
  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    // LONGTERM implement
    throw new RuntimeException();
  }
}
