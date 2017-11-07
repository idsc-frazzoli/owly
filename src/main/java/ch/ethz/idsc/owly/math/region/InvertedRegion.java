// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.Tensor;

public class InvertedRegion implements TensorRegion {
  private final TensorRegion region;

  public InvertedRegion(TensorRegion region) {
    this.region = region;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return !region.isMember(tensor);
  }
}
