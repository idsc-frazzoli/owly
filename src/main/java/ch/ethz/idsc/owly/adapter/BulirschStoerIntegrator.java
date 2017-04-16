// code by jph
package ch.ethz.idsc.owly.adapter;

import ch.ethz.idsc.owly.util.Integrator;
import ch.ethz.idsc.owly.util.StateSpaceModel;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class BulirschStoerIntegrator implements Integrator {
  @Override
  public Tensor step(StateSpaceModel stateSpaceModel, Tensor x, Tensor u, Scalar dt) {
    // TODO implementation
    return null;
  }
}
