// code by jph
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.noise.SimplexNoise;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;

public class R2NoiseRegion implements Region {
  private final double threshold;

  public R2NoiseRegion(double threshold) {
    this.threshold = threshold;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return threshold < SimplexNoise.at( //
        tensor.Get(0).number().doubleValue(), //
        tensor.Get(1).number().doubleValue());
  }
}
