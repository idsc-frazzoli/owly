// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.tensor.Tensor;

class QueueRender extends AbstractRender {
  Collection<GlcNode> collection;

  QueueRender(Collection<GlcNode> collection) {
    this.collection = collection;
  }

  @Override
  void render(AbstractLayer abstractLayer, Graphics2D graphics) {
    {
      graphics.setColor(new Color(0, 192, 192, 128));
      for (GlcNode node : collection) {
        Tensor x = node.stateTime().x();
        Point2D p = abstractLayer.toPoint2D(x);
        Shape shape = new Rectangle2D.Double(p.getX() - 1, p.getY() - 1, 3, 3);
        graphics.fill(shape);
      }
    }
    {
      // FIXME
      // Node node = trajectoryPlanner.peek();
      // if (node != null) {
      // int radius = 3;
      // graphics.setColor(new Color(255, 128, 128, 255));
      // Point2D point2d = toPoint2D(node.stateTime().x());
      // graphics.draw(new Ellipse2D.Double(point2d.getX() - radius, point2d.getY() - radius, radius * 2, radius * 2));
      // }
    }
  }
}
