// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface Integrator {
  Tensor step(StateSpaceModel stateSpaceModel, Tensor x, Tensor u, Scalar dt);
}
