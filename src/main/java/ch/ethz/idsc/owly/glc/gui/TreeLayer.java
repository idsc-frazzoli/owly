// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.DoubleSummaryStatistics;

import ch.ethz.idsc.owly.glc.core.Node;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.Scalar;

class TreeLayer extends AbstractLayer {
  TreeLayer(GlcComponent glcComponent) {
    super(glcComponent);
  }

  @Override
  void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner) {
    DoubleSummaryStatistics dss = trajectoryPlanner.getNodes().stream() //
        .map(n -> n.cost) //
        .map(Scalar::number) //
        .mapToDouble(Number::doubleValue) //
        .filter(Double::isFinite) //
        .summaryStatistics();
    final double min = dss.getMin();
    final double max = dss.getMax();
    graphics.setColor(Color.BLUE);
    for (Node node : trajectoryPlanner.getNodes()) {
      double val = node.cost.number().doubleValue();
      final double interp = (val - min) / (max - min);
      graphics.setColor(new Hue(interp, 1, 1, 1).rgba);
      final Point2D p1 = toPoint2D(node.x);
      graphics.fill(new Rectangle2D.Double(p1.getX(), p1.getY(), 1, 1));
      if (!node.isRoot()) {
        Node parent = node.parent();
        Point2D p2 = toPoint2D(parent.x);
        graphics.setColor(new Hue(interp, 1, 1, .1).rgba);
        Shape shape = new Line2D.Double(p1.getX(), p1.getY(), p2.getX(), p2.getY());
        graphics.draw(shape);
      }
    }
  }
}
