// code by bapaden and jph
package ch.ethz.idsc.owly.adapter;

import ch.ethz.idsc.owly.util.Integrator;
import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** Numerical Recipes 3rd Edition (17.1.1) */
public final class EulerIntegrator implements Integrator {
  @Override
  public Tensor step(StateSpaceModel stateSpaceModel, Tensor x, Tensor u, Scalar dt) {
    Tensor k1 = stateSpaceModel.flow(x, u).multiply(dt);
    return x.add(k1);
  }
}
