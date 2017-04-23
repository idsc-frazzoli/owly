// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Tensor;

public class EmptyRegion implements Region {
  @Override
  public boolean isMember(Tensor tensor) {
    return false;
  }
}
