// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import ch.ethz.idsc.owly.glc.core.Node;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.Tensor;

class QueueLayer extends AbstractLayer {
  QueueLayer(GlcComponent glcComponent) {
    super(glcComponent);
  }

  @Override
  void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner) {
    {
      graphics.setColor(new Color(0, 192, 192, 128));
      for (Node node : trajectoryPlanner.getQueue()) {
        Tensor x = node.stateTime().x();
        Point2D p = toPoint2D(x);
        Shape shape = new Rectangle2D.Double(p.getX() - 1, p.getY() - 1, 3, 3);
        graphics.fill(shape);
      }
    }
    {
      Node node = trajectoryPlanner.peek();
      if (node != null) {
        int radius = 3;
        graphics.setColor(new Color(255, 128, 128, 255));
        Point2D point2d = toPoint2D(node.stateTime().x());
        graphics.draw(new Ellipse2D.Double(point2d.getX() - radius, point2d.getY() - radius, radius*2, radius*2));
      }
    }
  }
}
