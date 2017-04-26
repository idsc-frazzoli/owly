// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.Region;
import ch.ethz.idsc.owly.tree.Cluster;
import ch.ethz.idsc.owly.tree.DistanceInterface;
import ch.ethz.idsc.owly.tree.NdTreeMap;
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
    ndTreeMap = new NdTreeMap<>(lbounds, ubounds, 10, 5); // TODO
    for (Tensor point : points)
      ndTreeMap.add(point, point.toString()); // TODO
    this.radius = radius;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    Cluster<String> cluster = ndTreeMap.buildCluster(tensor, 1, DistanceInterface.EUCLIDEAN);
    Scalar dist = cluster.iterator().next().distanceToCenter;
    return Scalars.lessEquals(dist, radius);
  }
}
