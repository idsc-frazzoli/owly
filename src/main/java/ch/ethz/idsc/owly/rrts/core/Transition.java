// code by jph
package ch.ethz.idsc.owly.rrts.core;

import java.util.List;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public interface Transition {
  /** @return start state of this transition */
  Tensor start();

  /** @return end state of this transition */
  Tensor end();

  /** @return length of this transition */
  Scalar length();

  /** @param t0 time at start()
   * @param ofs is non-negative and strictly less than dt
   * @param dt
   * @return */
  List<StateTime> sampled(Scalar t0, Scalar ofs, Scalar dt);

  StateTime splitAt(Scalar t1);
}
