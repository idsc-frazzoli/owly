// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.DoubleSummaryStatistics;

import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.tensor.Scalar;

class TreeLayer extends AbstractLayer {
  Collection<? extends StateCostNode> collection;

  TreeLayer(OwlyComponent glcComponent) {
    super(glcComponent);
  }

  void setCollection(Collection<? extends StateCostNode> collection) {
    this.collection = collection;
  }

  @Override
  void render(Graphics2D graphics) {
    DoubleSummaryStatistics dss = collection.stream() //
        .map(n -> n.costFromRoot()) //
        .map(Scalar::number) //
        .mapToDouble(Number::doubleValue) //
        .filter(Double::isFinite) //
        .summaryStatistics();
    final double min = dss.getMin();
    final double max = dss.getMax();
    graphics.setColor(Color.BLUE);
    for (StateCostNode node : collection) {
      double val = node.costFromRoot().number().doubleValue();
      final double interp = (val - min) / (max - min);
      graphics.setColor(new Hue(interp, 1, 1, 1).rgba);
      final Point2D p1 = toPoint2D(node.state());
      graphics.fill(new Rectangle2D.Double(p1.getX(), p1.getY(), 1, 1));
      StateCostNode parent = node.parent();
      if (parent != null) {
        Point2D p2 = toPoint2D(parent.state());
        graphics.setColor(new Hue(interp, 1, 1, .1).rgba);
        Shape shape = new Line2D.Double(p1.getX(), p1.getY(), p2.getX(), p2.getY());
        graphics.draw(shape);
      }
    }
  }
}