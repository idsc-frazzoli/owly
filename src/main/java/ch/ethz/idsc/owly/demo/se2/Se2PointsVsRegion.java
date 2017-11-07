// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.region.TensorRegion;
import ch.ethz.idsc.owly.math.se2.Se2Utils;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/** used in se2 animation demo to check if footprint of vehicle intersects with obstacle region */
public class Se2PointsVsRegion implements TensorRegion {
  private final Tensor points;
  private final TensorRegion region;

  public Se2PointsVsRegion(Tensor points, TensorRegion region) {
    this.points = Tensor.of(points.stream().map(row -> row.append(RealScalar.ONE))).unmodifiable();
    this.region = region;
  }

  /** @param tensor of the form (x,y,theta)
   * @return true if any of the points subject to the given transformation are in region */
  @Override
  public boolean isMember(Tensor tensor) {
    Tensor matrix = Se2Utils.toSE2Matrix(tensor);
    return points.stream() //
        .map(point -> matrix.dot(point)) //
        .anyMatch(region::isMember);
  }

  public Tensor points() {
    return points;
  }
}
