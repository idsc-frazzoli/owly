// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Graphics2D;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.region.PolygonRegion;
import ch.ethz.idsc.tensor.Tensor;

public class PolygonRegionRender implements RenderInterface {
  private final Tensor polygon;

  public PolygonRegionRender(PolygonRegion polygonRegion) {
    polygon = polygonRegion.polygon();
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    graphics.setColor(RegionHelper.COLOR);
    graphics.fill(geometricLayer.toPath2D(polygon));
  }
}
