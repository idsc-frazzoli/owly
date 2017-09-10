// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/**
 * 
 */
public class Se2PointsVsRegion implements Region {
  private final Tensor points;
  private final Region region;

  public Se2PointsVsRegion(Tensor points, Region region) {
    this.points = Tensor.of(points.stream().map(row -> row.append(RealScalar.ONE))).unmodifiable();
    this.region = region;
  }

  /** @param tensor of the form (x,y,theta)
   * @return true if any of the points subject to the given transformation are in region */
  @Override
  public boolean isMember(Tensor tensor) {
    return points.dot(Se2Utils.toSE2MatrixTranspose(tensor)).stream().anyMatch(region::isMember);
  }

  public Tensor points() {
    return points;
  }
}
