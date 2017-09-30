// code by jph
package ch.ethz.idsc.owly.gui.ren;

import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.DoubleSummaryStatistics;
import java.util.Objects;

import ch.ethz.idsc.owly.data.tree.StateCostNode;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.misc.ColorLookup;
import ch.ethz.idsc.tensor.Scalar;

/** renders the edges between nodes
 * 
 * the edges are drawn as straight lines with the color of the cost to root */
public class TreeRender implements RenderInterface {
  private static final int NODE_WIDTH = 2;
  // ---
  private Collection<? extends StateCostNode> collection;
  private final ColorLookup nodeColor = ColorLookup.hsluv_lightness(.50, 1.0);
  private final ColorLookup edgeColor = ColorLookup.hsluv_lightness(.65, 0.3);

  public TreeRender(Collection<? extends StateCostNode> collection) {
    this.collection = collection;
  }

  @Override
  public void render(GeometricLayer owlyLayer, Graphics2D graphics) {
    Collection<? extends StateCostNode> _collection = collection;
    if (Objects.isNull(_collection))
      return;
    DoubleSummaryStatistics dss = _collection.stream() //
        .map(StateCostNode::costFromRoot) //
        .map(Scalar::number) //
        .mapToDouble(Number::doubleValue) //
        .filter(Double::isFinite) //
        .summaryStatistics();
    final double min = dss.getMin();
    final double max = dss.getMax();
    for (StateCostNode node : _collection) {
      double val = node.costFromRoot().number().doubleValue();
      if (!Double.isFinite(val))
        throw new RuntimeException("cost from root " + val);
      final double interp = (val - min) / (max - min);
      graphics.setColor(nodeColor.get(interp));
      final Point2D p1 = owlyLayer.toPoint2D(node.state());
      graphics.fill(new Rectangle2D.Double(p1.getX(), p1.getY(), NODE_WIDTH, NODE_WIDTH));
      StateCostNode parent = node.parent();
      if (Objects.nonNull(parent)) {
        Point2D p2 = owlyLayer.toPoint2D(parent.state());
        graphics.setColor(edgeColor.get(interp));
        Shape shape = new Line2D.Double(p1.getX(), p1.getY(), p2.getX(), p2.getY());
        graphics.draw(shape);
      }
    }
  }

  public void setCollection(Collection<? extends StateCostNode> collection) {
    this.collection = collection;
  }
}
