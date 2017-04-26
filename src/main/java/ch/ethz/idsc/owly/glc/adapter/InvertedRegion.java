// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.Region;
import ch.ethz.idsc.tensor.Tensor;

public class InvertedRegion implements Region {
  final Region region;

  public InvertedRegion(Region region) {
    this.region = region;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return !region.isMember(tensor);
  }
}
