// code by jph
package ch.ethz.idsc.owly.adapter;

import ch.ethz.idsc.owly.util.Integrator;
import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** Numerical Recipes 3rd Edition (17.1.2) */
public final class MidpointIntegrator implements Integrator {
  private static final Scalar HALF = RationalScalar.of(1, 2);

  @Override
  public Tensor step(StateSpaceModel stateSpaceModel, Tensor x, Tensor u, Scalar dt) {
    Tensor k1 = stateSpaceModel.flow(x, u).multiply(dt);
    Tensor k2 = stateSpaceModel.flow(x.add(k1.multiply(HALF)), u).multiply(dt);
    return x.add(k2);
  }
}
