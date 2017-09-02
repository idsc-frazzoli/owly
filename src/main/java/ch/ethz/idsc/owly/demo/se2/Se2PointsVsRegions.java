// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Transpose;

public enum Se2PointsVsRegions {
  ;
  /** @param x_coords
   * @param region
   * @return */
  public static Se2PointsVsRegion line(Tensor x_coords, Region region) {
    return new Se2PointsVsRegion( //
        Transpose.of(Tensors.of(x_coords, Array.zeros(x_coords.length()))), region);
  }
}
