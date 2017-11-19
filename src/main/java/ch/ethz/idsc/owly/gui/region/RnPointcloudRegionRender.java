// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Graphics2D;
import java.awt.geom.Path2D;

import ch.ethz.idsc.owly.demo.rn.RnPointcloudRegion;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.r2.CirclePoints;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

class RnPointcloudRegionRender implements RenderInterface {
  private static final int RESOLUTION = 16;
  // ---
  private final Tensor points;
  private final Tensor polygon;

  public RnPointcloudRegionRender(RnPointcloudRegion rnPointcloudRegion) {
    points = rnPointcloudRegion.points();
    Scalar radius = rnPointcloudRegion.radius();
    polygon = CirclePoints.elliptic(RESOLUTION, radius, radius);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    graphics.setColor(RegionRenders.COLOR);
    for (Tensor center : points) {
      Path2D path2D = geometricLayer.toPath2D(Tensor.of(polygon.stream().map(row -> row.add(center))));
      graphics.setColor(RegionRenders.COLOR);
      graphics.fill(path2D);
      graphics.setColor(RegionRenders.BOUNDARY);
      path2D.closePath();
      graphics.draw(path2D);
    }
  }
}
