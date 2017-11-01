// code by jph
package ch.ethz.idsc.owly.math.noise;

import ch.ethz.idsc.tensor.RealScalar;

public enum ContinuousNoiseUtils {
  ;
  public static ContinuousNoise wrap2D(NativeContinuousNoise nativeContinuousNoise) {
    return tensor -> RealScalar.of(nativeContinuousNoise.at( //
        tensor.Get(0).number().doubleValue(), //
        tensor.Get(1).number().doubleValue()));
  }

  public static ContinuousNoise wrap3D(NativeContinuousNoise nativeContinuousNoise) {
    return tensor -> RealScalar.of(nativeContinuousNoise.at( //
        tensor.Get(0).number().doubleValue(), //
        tensor.Get(1).number().doubleValue(), //
        tensor.Get(2).number().doubleValue()));
  }
}
