// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Graphics2D;
import java.awt.geom.Path2D;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.CirclePoints;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.tensor.Tensor;

class EllipsoidRegionRender implements RenderInterface {
  private static final int RESOLUTION = 22;
  // ---
  private final Tensor polygon;

  public EllipsoidRegionRender(EllipsoidRegion ellipsoidRegion) {
    Tensor center = ellipsoidRegion.center();
    Tensor radius = ellipsoidRegion.radius();
    polygon = Tensor.of(CirclePoints.elliptic(RESOLUTION, radius.Get(0), radius.Get(1)) //
        .stream().map(row -> row.add(center)));
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Path2D path2D = geometricLayer.toPath2D(polygon);
    graphics.setColor(RegionRenders.COLOR);
    graphics.fill(path2D);
    graphics.setColor(RegionRenders.BOUNDARY);
    path2D.closePath();
    graphics.draw(path2D);
  }
}
