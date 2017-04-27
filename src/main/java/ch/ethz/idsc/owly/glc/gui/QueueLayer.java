// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
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
    int rgb = 64;
    graphics.setColor(new Color(rgb, rgb, rgb, 128));
    for (Node node : trajectoryPlanner.getQueue()) {
      Tensor x = node.x;
      Point2D p = toPoint2D(x);
      Shape shape = new Rectangle2D.Double(p.getX() - 1, p.getY() - 1, 3, 3);
      graphics.fill(shape);
    }
  }
}
