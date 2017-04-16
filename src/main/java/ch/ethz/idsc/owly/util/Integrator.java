// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface Integrator {
  Tensor step(Flow flow, Tensor x, Scalar dt);
}
