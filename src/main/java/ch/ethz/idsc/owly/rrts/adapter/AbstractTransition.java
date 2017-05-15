// code by jph
package ch.ethz.idsc.owly.rrts.adapter;

import ch.ethz.idsc.owly.rrts.core.Transition;
import ch.ethz.idsc.tensor.Tensor;

/** suggested base class for all implementations of {@link Transition} */
public abstract class AbstractTransition implements Transition {
  private final Tensor start;
  private final Tensor end;

  public AbstractTransition(Tensor start, Tensor end) {
    this.start = start.unmodifiable();
    this.end = end.unmodifiable();
  }

  @Override
  public final Tensor start() {
    return start;
  }

  @Override
  public final Tensor end() {
    return end;
  }
}
