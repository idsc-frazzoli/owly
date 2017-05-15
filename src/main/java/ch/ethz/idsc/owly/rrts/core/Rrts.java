// code by jph
package ch.ethz.idsc.owly.rrts.core;

import ch.ethz.idsc.tensor.Tensor;

public interface Rrts {
  RrtsNode insertAsNode(Tensor state, int k_nearest);

  void rewireAround(RrtsNode rrtsNode, int k_nearest);

  int rewireCount();
}
