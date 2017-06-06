// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.Collection;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.ConvexHull;

class GoalRender implements AbstractRender {
  private final Collection<StateTime> collection;

  GoalRender(Collection<StateTime> collection) {
    this.collection = collection;
  }

  @Override
  public void render(OwlyLayer abstractLayer, Graphics2D graphics) {
    { // draw convex hull of goal points
      Tensor points = Tensor.of(collection.stream().map(StateTime::x).map(x -> x.extract(0, 2)));
      if (2 < points.length()) {
        Tensor hull = ConvexHull.of(points);
        Path2D path2d = new Path2D.Double();
        boolean init = false;
        for (Tensor p : hull) {
          Point2D point2d = abstractLayer.toPoint2D(p);
          if (init) {
            path2d.lineTo(point2d.getX(), point2d.getY());
          } else {
            path2d.moveTo(point2d.getX(), point2d.getY());
            init = true;
          }
        }
        graphics.setColor(new Color(224, 168, 0, 128));
        graphics.fill(path2d);
      }
    }
    { // draw discovered points
      double radius = 9;
      double offset = -radius * 0.5;
      graphics.setColor(new Color(224, 168, 0, 224));
      for (StateTime stateTime : collection) {
        Point2D point2d = abstractLayer.toPoint2D(stateTime.x());
        graphics.draw(new Ellipse2D.Double(point2d.getX() + offset, point2d.getY() + offset, radius, radius));
      }
    }
  }
}
