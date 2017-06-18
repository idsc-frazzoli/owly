// code by jph
package ch.ethz.idsc.owly.demo.rn.rrts;

import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.owly.rrts.core.TransitionSpace;
import ch.ethz.idsc.tensor.Tensor;

class RnTransitionSpace implements TransitionSpace {
  @Override
  public Transition connect(Tensor start, Tensor end) {
    return new RnTransition(start, end);
  }
}
