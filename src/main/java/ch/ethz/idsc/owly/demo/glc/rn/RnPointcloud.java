// code by jph
package ch.ethz.idsc.owly.demo.glc.rn;

import ch.ethz.idsc.owly.math.EmptyRegion;
import ch.ethz.idsc.owly.math.Region;
import ch.ethz.idsc.owly.tree.Cluster;
import ch.ethz.idsc.owly.tree.Distancer;
import ch.ethz.idsc.owly.tree.NdTreeMap;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Min;

public class RnPointcloud implements Region {
  public static Region create(Tensor points, Scalar radius) {
    return points.length() == 0 ? new EmptyRegion() : new RnPointcloud(points, radius);
  }

  final NdTreeMap<String> ndTreeMap;
  final Scalar radius;

  private RnPointcloud(Tensor points, Scalar radius) {
    Tensor pt = Transpose.of(points);
    Tensor lbounds = Tensors.vector(i -> pt.get(i).flatten(0).reduce(Min::of).get(), pt.length());
    Tensor ubounds = Tensors.vector(i -> pt.get(i).flatten(0).reduce(Max::of).get(), pt.length());
    // System.out.println("---");
    // System.out.println(lbounds);
    // System.out.println(ubounds);
    ndTreeMap = new NdTreeMap<>(lbounds, ubounds, 10, 5);
    for (Tensor point : points)
      ndTreeMap.add(point, point.toString()); // TODO
    this.radius = radius;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    Cluster<String> cluster = ndTreeMap.buildCluster(tensor, 1, Distancer.EUCLIDEAN);
    Scalar dist = cluster.iterator().next().distanceToCenter;
    return Scalars.lessEquals(dist, radius);
  }
}
