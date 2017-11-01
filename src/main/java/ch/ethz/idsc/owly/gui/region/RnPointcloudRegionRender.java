// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;

import ch.ethz.idsc.owly.demo.rn.RnPointcloudRegion;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.se2.Se2Utils;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class RnPointcloudRegionRender implements RenderInterface {
  private final Tensor points;
  private final Scalar radius;

  public RnPointcloudRegionRender(RnPointcloudRegion rnPointcloudRegion) {
    points = rnPointcloudRegion.points();
    radius = rnPointcloudRegion.radius();
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    AffineTransform ori = graphics.getTransform();
    AffineTransform at = Se2Utils.toAffineTransform(geometricLayer.getMatrix());
    graphics.setTransform(at);
    graphics.setColor(RegionRenders.COLOR);
    double rad = 2 * radius.number().doubleValue();
    for (Tensor center : points) {
      Tensor nw = center.map(scalar -> scalar.subtract(radius));
      graphics.fill(new Ellipse2D.Double( //
          nw.Get(0).number().doubleValue(), //
          nw.Get(1).number().doubleValue(), //
          rad, rad));
    }
    graphics.setTransform(ori);
  }
}
