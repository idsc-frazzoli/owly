// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.math.Polygons;
import ch.ethz.idsc.tensor.Tensor;

/** check if input tensor is inside a polygon */
public class PolygonRegion implements Region {
  private final Tensor polygon;

  public PolygonRegion(Tensor polygon) {
    this.polygon = polygon.copy();
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return Polygons.isInside(polygon, tensor);
  }
}
