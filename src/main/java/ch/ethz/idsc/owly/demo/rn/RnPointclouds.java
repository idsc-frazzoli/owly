// code by jph and jl
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.sample.BoxRandomSample;
import ch.ethz.idsc.owly.math.sample.RandomSample;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;

public enum RnPointclouds {
  ;
  /** @param num number of points
   * @param width of area, in which they are created
   * @param offset of area, in which they are created
   * @param radius of each obstacle
   * @return region with random points as obstacles */
  public static Region createRandomRegion(int num, Tensor offset, Tensor width, Scalar radius) {
    RandomSample randomSample = new BoxRandomSample(offset, offset.add(width));
    Tensor points = Array.of(list -> randomSample.nextSample(), num);
    return RnPointcloudRegion.of(points, radius);
  }
}
