// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.se2.Se2Utils;
import ch.ethz.idsc.tensor.Tensor;

// TODO the rendering is inaccurate 
public class EllipsoidRegionRender implements RenderInterface {
  private final EllipsoidRegion ellipsoidRegion;

  public EllipsoidRegionRender(EllipsoidRegion ellipsoidRegion) {
    this.ellipsoidRegion = ellipsoidRegion;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Tensor center = ellipsoidRegion.center().extract(0, 2);
    Tensor radius = ellipsoidRegion.radius().extract(0, 2);
    Tensor model2pixel = geometricLayer.getMatrix();
    AffineTransform at = Se2Utils.toAffineTransform(model2pixel);
    AffineTransform ori = graphics.getTransform();
    graphics.setTransform(at);
    graphics.setColor(RegionHelper.COLOR);
    Tensor nw = center.subtract(radius);
    graphics.fill(new Ellipse2D.Double( //
        nw.Get(0).number().doubleValue(), //
        nw.Get(1).number().doubleValue(), //
        2 * radius.Get(0).number().doubleValue(), //
        2 * radius.Get(1).number().doubleValue()));
    graphics.setTransform(ori);
  }
}
