// code by jph
package ch.ethz.idsc.owly.math.noise;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public enum ContinuousNoiseUtils {
  ;
  public static ContinuousNoise wrap2D(NativeContinuousNoise nativeContinuousNoise) {
    return new ContinuousNoise() {
      @Override
      public Scalar apply(Tensor tensor) {
        return RealScalar.of(nativeContinuousNoise.at( //
            tensor.Get(0).number().doubleValue(), //
            tensor.Get(1).number().doubleValue()));
      }
    };
  }

  public static ContinuousNoise wrap3D(NativeContinuousNoise nativeContinuousNoise) {
    return new ContinuousNoise() {
      @Override
      public Scalar apply(Tensor tensor) {
        return RealScalar.of(nativeContinuousNoise.at( //
            tensor.Get(0).number().doubleValue(), //
            tensor.Get(1).number().doubleValue(), //
            tensor.Get(2).number().doubleValue()));
      }
    };
  }
}
