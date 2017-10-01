// code by jph
package ch.ethz.idsc.owly.gui.ren;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Map;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

class DomainRender implements RenderInterface {
  private static final Color SHADING = new Color(192, 192, 192, 64);
  // ---
  private final Map<Tensor, GlcNode> map;
  private final Tensor eta_invert;

  DomainRender(Map<Tensor, GlcNode> map, Tensor eta) {
    this.map = map;
    this.eta_invert = eta.extract(0, 2).map(Scalar::reciprocal);
  }

  @Override
  public void render(GeometricLayer owlyLayer, Graphics2D graphics) {
    graphics.setColor(SHADING);
    Point2D h1 = owlyLayer.toPoint2D(Tensors.vector(0, 0));
    Point2D h2 = owlyLayer.toPoint2D(eta_invert);
    double w = h2.getX() - h1.getX() - 1;
    double h = h1.getY() - h2.getY() - 1;
    if (w < 0 || h < 0)
      return;
    map.keySet().stream().map(t -> t.extract(0, 2)).distinct().forEach(key -> {
      Tensor x = key.pmul(eta_invert);
      Point2D p = owlyLayer.toPoint2D(x);
      Shape shape = new Rectangle2D.Double(p.getX(), p.getY() - h, w, h);
      graphics.fill(shape);
    });
  }
}
