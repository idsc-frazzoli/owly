// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.Collection;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.tensor.Tensor;

class QueueRender implements AbstractRender {
  private Collection<GlcNode> collection;

  QueueRender(Collection<GlcNode> collection) {
    this.collection = collection;
  }

  @Override
  public void render(OwlyLayer abstractLayer, Graphics2D graphics) {
    graphics.setColor(new Color(0, 192, 192, 128));
    for (GlcNode node : collection) {
      Tensor x = node.stateTime().x();
      Point2D p = abstractLayer.toPoint2D(x);
      Shape shape2 = new Ellipse2D.Double(p.getX() - 4, p.getY() - 4, 8, 8);
      graphics.fill(shape2);
    }
  }
}
