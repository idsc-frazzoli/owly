// code by jph
package ch.ethz.idsc.owl.math.noise;

import java.util.stream.IntStream;

public class LayeredContinuousNoise implements NativeContinuousNoise {
  /** WARNING: values are not copied but used by reference
   *
   * @param nativeContinuousNoise
   * @param magnitude
   * @param frequency
   * @return */
  public static NativeContinuousNoise of(NativeContinuousNoise nativeContinuousNoise, double[] magnitude, double[] frequency) {
    return new LayeredContinuousNoise(nativeContinuousNoise, magnitude, frequency);
  }

  // ---
  private final NativeContinuousNoise nativeContinuousNoise;
  private final double[] magnitude;
  private final double[] frequency;

  private LayeredContinuousNoise(NativeContinuousNoise nativeContinuousNoise, double[] magnitude, double[] frequency) {
    if (magnitude.length != frequency.length)
      throw new RuntimeException();
    this.nativeContinuousNoise = nativeContinuousNoise;
    this.magnitude = magnitude;
    this.frequency = frequency;
  }

  @Override
  public double at(double x) {
    return at(x, 0);
  }

  @Override
  public double at(double x, double y) {
    return IntStream.range(0, magnitude.length).parallel() //
        .mapToDouble(index -> magnitude[index] * nativeContinuousNoise.at( //
            x * frequency[index], //
            y * frequency[index])) //
        .sum();
  }

  @Override
  public double at(double x, double y, double z) {
    return IntStream.range(0, magnitude.length).parallel() //
        .mapToDouble(index -> magnitude[index] * nativeContinuousNoise.at( //
            x * frequency[index], //
            y * frequency[index], //
            z * frequency[index])) //
        .sum();
  }
}
