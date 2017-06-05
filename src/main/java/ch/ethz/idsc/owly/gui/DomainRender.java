// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Map;
import java.util.Map.Entry;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

class DomainRender implements AbstractRender {
  private final Map<Tensor, GlcNode> map;
  Tensor eta_invert;

  DomainRender(Map<Tensor, GlcNode> map, Tensor eta) {
    this.map = map;
    this.eta_invert = eta.map(Scalar::invert);
  }

  @Override
  public void render(AbstractLayer abstractLayer, Graphics2D graphics) {
    graphics.setColor(new Color(128, 128, 128, 128));
    for (Entry<Tensor, GlcNode> entry : map.entrySet()) {
      Tensor x = entry.getKey().pmul(eta_invert);
      // Tensor x = node.stateTime().x();
      Point2D p = abstractLayer.toPoint2D(x);
      Shape shape = new Rectangle2D.Double(p.getX() - 1, p.getY() - 1, 2, 2);
      graphics.fill(shape);
    }
  }
}
