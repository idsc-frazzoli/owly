// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.Tensor;

public class InvertedRegion implements Region {
  private final Region region;

  public InvertedRegion(Region region) {
    this.region = region;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return !region.isMember(tensor);
  }
}
