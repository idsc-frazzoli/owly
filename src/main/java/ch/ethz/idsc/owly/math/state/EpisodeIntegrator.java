// code by jph
package ch.ethz.idsc.owly.math.state;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** produces a sequence of {@link StateTime} that progresses as control input is given
 * 
 * {@link EpisodeIntegrator} stores the previous state from which it continuously
 * extends the trajectory */
public interface EpisodeIntegrator {
  /** @param u constant control input during the time of integration
   * @param now absolute point in time
   * @return */
  StateTime move(Tensor u, Scalar now);
}
