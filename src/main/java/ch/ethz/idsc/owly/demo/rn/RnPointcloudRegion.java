// code by jph
package ch.ethz.idsc.owly.demo.rn;

import ch.ethz.idsc.owly.data.nd.NdCenterInterface;
import ch.ethz.idsc.owly.data.nd.NdCluster;
import ch.ethz.idsc.owly.data.nd.NdMap;
import ch.ethz.idsc.owly.data.nd.NdTreeMap;
import ch.ethz.idsc.owly.math.region.EmptyRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Min;

public class RnPointcloudRegion implements Region {
  /** @param points
   * @param radius
   * @return */
  public static Region of(Tensor points, Scalar radius) {
    return points.length() == 0 ? EmptyRegion.INSTANCE : new RnPointcloudRegion(points, radius);
  }

  // ---
  private final NdMap<Void> ndMap;
  private final Scalar radius;

  private RnPointcloudRegion(Tensor points, Scalar radius) {
    Tensor pt = Transpose.of(points);
    Tensor lbounds = Tensors.vector(i -> pt.get(i).stream().reduce(Min::of).get(), pt.length());
    Tensor ubounds = Tensors.vector(i -> pt.get(i).stream().reduce(Max::of).get(), pt.length());
    // System.out.println("---");
    // System.out.println(lbounds);
    // System.out.println(ubounds);
    ndMap = new NdTreeMap<>(lbounds, ubounds, 5, 20);
    for (Tensor point : points)
      ndMap.add(point, null);
    this.radius = radius;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    NdCenterInterface distanceInterface = NdCenterInterface.euclidean(tensor);
    NdCluster<Void> ndCluster = ndMap.buildCluster(distanceInterface, 1);
    // System.out.println(ndCluster.considered() + " / " + ndMap.size());
    Scalar distance = ndCluster.collection().iterator().next().distance();
    return Scalars.lessEquals(distance, radius);
  }
}
