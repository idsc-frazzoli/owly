// code by jph
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.noise.ContinuousNoise;
import ch.ethz.idsc.owly.math.noise.ContinuousNoiseUtils;
import ch.ethz.idsc.owly.math.noise.SimplexContinuousNoise;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** {@link R2NoiseRegion} is an implicit function region.
 * 
 * the simplex noise function is a continuous bivariate function with values in the interval [-1, 1]
 * https://de.wikipedia.org/wiki/Simplex_Noise
 * 
 * membership in the region for coordinates (x,y) that evaluate the noise function above a given threshold. */
public class R2NoiseRegion implements Region {
  private static final ContinuousNoise CONTINUOUS_NOISE = ContinuousNoiseUtils.wrap2D(SimplexContinuousNoise.FUNCTION);
  // ---
  private final Scalar threshold;

  public R2NoiseRegion(Scalar threshold) {
    this.threshold = threshold;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return Scalars.lessThan(threshold, CONTINUOUS_NOISE.apply(tensor));
  }
}
