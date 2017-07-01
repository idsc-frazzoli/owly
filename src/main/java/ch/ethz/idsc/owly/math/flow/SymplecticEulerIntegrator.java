// code by jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

enum SymplecticEulerIntegrator implements Integrator {
  INSTANCE;
  // ---
  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    // LONGTERM implement
    return null;
  }
}
