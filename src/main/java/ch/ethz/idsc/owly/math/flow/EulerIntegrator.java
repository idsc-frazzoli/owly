// code by bapaden and jph
package ch.ethz.idsc.owly.math.flow;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** Numerical Recipes 3rd Edition (17.1.1) */
public enum EulerIntegrator implements Integrator {
  INSTANCE;
  // ---
  @Override
  public Tensor step(Flow flow, Tensor x, Scalar h) {
    Tensor k1 = flow.at(x).multiply(h);
    return x.add(k1);
  }
}
