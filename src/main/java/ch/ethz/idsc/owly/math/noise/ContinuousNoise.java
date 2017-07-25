// adapted by jph
package ch.ethz.idsc.owly.math.noise;

public interface ContinuousNoise {
  /** @param x
   * @param y
   * @param z
   * @return value in the interval [-1, 1] that varies smoothly with x, y, z */
  double at(double x, double y, double z);
}
