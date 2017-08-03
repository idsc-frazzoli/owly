// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.Tensor;

public enum EmptyRegion implements Region {
  INSTANCE;
  // ---
  @Override
  public boolean isMember(Tensor tensor) {
    return false;
  }
}
