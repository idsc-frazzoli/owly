// code by jph
package ch.ethz.idsc.owly.rrts.core;

import ch.ethz.idsc.tensor.Tensor;

public interface TransitionSpace {
  Transition connect(Tensor start, Tensor end);
}
