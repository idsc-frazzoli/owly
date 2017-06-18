// code by jph and jl
package ch.ethz.idsc.owly.demo.rn;

import java.util.Random;
import java.util.stream.IntStream;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public enum RnPointclouds {
  ;
  /** @param num number of points
   * @param width width of area, in which they are created
   * @param center center of area, in which they are created
   * @param radius radius of each obstacle
   * @return region with random points as obstacles */
  public static Region createRandomRegion(int num, Tensor width, Tensor center, Scalar radius) {
    Random random = new Random();
    Tensor points = Tensors.empty();
    IntStream.range(0, num).boxed() //
        .forEach(i -> points.append(width.pmul(Tensors.vector(random.nextDouble(), random.nextDouble()))//
            .subtract(width.multiply(RealScalar.of(0.5)).subtract(center))));
    return RnPointcloudRegion.of(points, radius);
  }
}
