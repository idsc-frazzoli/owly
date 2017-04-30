// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Tensor;

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
