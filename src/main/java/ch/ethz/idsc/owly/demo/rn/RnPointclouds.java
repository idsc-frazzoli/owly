// code by jph and jl
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.pdf.Distribution;
import ch.ethz.idsc.tensor.pdf.RandomVariate;
import ch.ethz.idsc.tensor.pdf.UniformDistribution;

public enum RnPointclouds {
  ;
  /** @param num number of points
   * @param width of area, in which they are created
   * @param offset of area, in which they are created
   * @param radius of each obstacle
   * @return region with random points as obstacles */
  public static Region createRandomRegion(int num, Tensor offset, Tensor width, Scalar radius) {
    return RnPointcloudRegion.of(randomPoints(num, offset, width), radius);
  }

  /* package */ static Tensor randomPoints(int num, Tensor offset, Tensor width) {
    Tensor points = Tensors.empty();
    Distribution distribution = UniformDistribution.unit();
    for (int index = 0; index < num; ++index) {
      Tensor rand = RandomVariate.of(distribution, width.length());
      Tensor point = width.pmul(rand).add(offset);
      points.append(point);
    }
    return points;
  }
}
