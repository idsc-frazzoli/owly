// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import java.util.Random;
import java.util.stream.IntStream;

import ch.ethz.idsc.owly.data.nd.NdCluster;
import ch.ethz.idsc.owly.data.nd.NdDistanceInterface;
import ch.ethz.idsc.owly.data.nd.NdTreeMap;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Min;

public class RnPointcloudRegion implements Region {
  public static Region create(Tensor points, Scalar radius) {
    return points.length() == 0 ? new EmptyRegion() : new RnPointcloudRegion(points, radius);
  }

  final NdTreeMap<String> ndTreeMap;
  final Scalar radius;

  private RnPointcloudRegion(Tensor points, Scalar radius) {
    Tensor pt = Transpose.of(points);
    Tensor lbounds = Tensors.vector(i -> pt.get(i).flatten(0).reduce(Min::of).get(), pt.length());
    Tensor ubounds = Tensors.vector(i -> pt.get(i).flatten(0).reduce(Max::of).get(), pt.length());
    // System.out.println("---");
    // System.out.println(lbounds);
    // System.out.println(ubounds);
    ndTreeMap = new NdTreeMap<>(lbounds, ubounds, 10, 5); // TODO magic const
    for (Tensor point : points)
      ndTreeMap.add(point, point.toString()); // TODO different key
    this.radius = radius;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    NdCluster<String> ndCluster = ndTreeMap.buildCluster(tensor, 1, NdDistanceInterface.EUCLIDEAN);
    Scalar distance = ndCluster.iterator().next().distanceToCenter;
    return Scalars.lessEquals(distance, radius);
  }

  /** @param num number of points
   * @param width width of area, in which they are created
   * @param center center of area, in which they are created
   * @param radius radius of each obstacle
   * @return the Pointcloudregion */
  public static Region createRandom(int num, Tensor width, Tensor center, Scalar radius) {
    Random random = new Random();
    Tensor points = Tensors.empty();
    IntStream.range(0, num).boxed() //
        .forEach(
            i -> points.append(width.pmul(Tensors.vector(random.nextDouble(), random.nextDouble()))//
                .subtract(width.multiply(RealScalar.of(0.5)).subtract(center))));
    return create(points, radius);
  }
}
