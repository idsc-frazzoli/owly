// code by jph
package ch.ethz.idsc.owly.rrts.core;

import ch.ethz.idsc.tensor.Tensor;

public interface Rrts {
  /** @param state
   * @param k_nearest
   * @return */
  RrtsNode insertAsNode(Tensor state, int k_nearest);

  /** @param rrtsNode
   * @param k_nearest */
  void rewireAround(RrtsNode rrtsNode, int k_nearest);

  /** @return number of times re-wiring was effective, i.e.
   * altered the parent of an existing node */
  int rewireCount();
}
