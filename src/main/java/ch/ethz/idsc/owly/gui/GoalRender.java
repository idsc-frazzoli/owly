// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.Collection;

import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.ConvexHull;

class GoalRender implements RenderInterface {
  private final Collection<StateTime> collection;

  GoalRender(Collection<StateTime> collection) {
    this.collection = collection;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // draw convex hull of goal points
      Tensor points = Tensor.of(collection.stream().map(StateTime::x).map(x -> x.extract(0, 2)));
      if (2 < points.length()) {
        graphics.setColor(new Color(224, 168, 0, 128));
        graphics.fill(owlyLayer.toPath2D(ConvexHull.of(points)));
      }
    }
    { // draw discovered points
      double radius = 9;
      double offset = -radius * 0.5;
      graphics.setColor(new Color(224, 168, 0, 224));
      for (StateTime stateTime : collection) {
        Point2D point2d = owlyLayer.toPoint2D(stateTime.x());
        graphics.draw(new Ellipse2D.Double(point2d.getX() + offset, point2d.getY() + offset, radius, radius));
      }
    }
  }
}
