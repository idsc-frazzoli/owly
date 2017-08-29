// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.DoubleSummaryStatistics;
import java.util.Objects;

import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.img.Hue;

/** renders the edges between nodes
 * 
 * the edges are drawn as straight lines with the color of the cost to root */
public class TreeRender implements RenderInterface {
  private static final int NODE_WIDTH = 2;
  private Collection<? extends StateCostNode> collection;

  public TreeRender(Collection<? extends StateCostNode> collection) {
    this.collection = collection;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    if (Objects.isNull(collection))
      return;
    DoubleSummaryStatistics dss = collection.stream() //
        .map(StateCostNode::costFromRoot) //
        .map(Scalar::number) //
        .mapToDouble(Number::doubleValue) //
        .filter(Double::isFinite) //
        .summaryStatistics();
    final double min = dss.getMin();
    final double max = dss.getMax();
    for (StateCostNode node : collection) {
      double val = node.costFromRoot().number().doubleValue();
      final double interp = (val - min) / (max - min);
      graphics.setColor(Hue.of(interp, 1, 1, 1));
      final Point2D p1 = owlyLayer.toPoint2D(node.state());
      graphics.fill(new Rectangle2D.Double(p1.getX(), p1.getY(), NODE_WIDTH, NODE_WIDTH));
      StateCostNode parent = node.parent();
      if (Objects.nonNull(parent)) {
        Point2D p2 = owlyLayer.toPoint2D(parent.state());
        graphics.setColor(Hue.of(interp, 1, 1, .2));
        Shape shape = new Line2D.Double(p1.getX(), p1.getY(), p2.getX(), p2.getY());
        graphics.draw(shape);
      }
    }
  }

  public void setCollection(Collection<? extends StateCostNode> collection) {
    this.collection = collection;
  }
}
