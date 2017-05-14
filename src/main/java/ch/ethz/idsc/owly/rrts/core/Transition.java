// code by jph
package ch.ethz.idsc.owly.rrts.core;

import java.util.List;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface Transition {
  Tensor start();

  Tensor end();

  Scalar length();

  /** @param t0 time at start()
   * @param ofs is non-negative and stricly less than dt
   * @param dt
   * @return */
  List<StateTime> sampled(Scalar t0, Scalar ofs, Scalar dt);

  StateTime splitAt(Scalar t1);
}
