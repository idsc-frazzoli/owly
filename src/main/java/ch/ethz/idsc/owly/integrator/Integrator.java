// code by jph
package ch.ethz.idsc.owly.integrator;

import ch.ethz.idsc.owly.util.Flow;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** integrator of time-invariant differential constraint */
public interface Integrator {
  /** @param flow
   * @param x
   * @param h
   * @return moves x along flow for time duration dt, i.e. "x + flow(x) * dt" */
  Tensor step(Flow flow, Tensor x, Scalar h);
}
