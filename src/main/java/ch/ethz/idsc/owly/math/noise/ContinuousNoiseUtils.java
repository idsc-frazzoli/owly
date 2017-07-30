// code by jph
package ch.ethz.idsc.owly.math.noise;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class ContinuousNoiseUtils implements ContinuousNoise {
  public static ContinuousNoise wrap(NativeContinuousNoise nativeContinuousNoise) {
    return new ContinuousNoiseUtils(nativeContinuousNoise);
  }

  // ---
  private final NativeContinuousNoise nativeContinuousNoise;

  private ContinuousNoiseUtils(NativeContinuousNoise nativeContinuousNoise) {
    this.nativeContinuousNoise = nativeContinuousNoise;
  }

  @Override
  public Scalar apply(Tensor tensor) {
    switch (tensor.length()) {
    case 2:
      return RealScalar.of(nativeContinuousNoise.at( //
          tensor.Get(0).number().doubleValue(), //
          tensor.Get(1).number().doubleValue()));
    case 3:
      return RealScalar.of(nativeContinuousNoise.at( //
          tensor.Get(0).number().doubleValue(), //
          tensor.Get(1).number().doubleValue(), //
          tensor.Get(2).number().doubleValue()));
    default:
      break;
    }
    throw new RuntimeException();
  }
}
