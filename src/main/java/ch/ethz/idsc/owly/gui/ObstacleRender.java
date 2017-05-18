// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owly.math.state.StateTime;

class ObstacleRender extends AbstractRender {
  Collection<StateTime> collection;

  ObstacleRender(Collection<StateTime> collection) {
    this.collection = collection;
  }

  @Override
  void render(AbstractLayer abstractLayer, Graphics2D graphics) {
    graphics.setColor(new Color(0, 0, 0, 128));
    for (StateTime stateTime : collection) {
      Point2D point2d = abstractLayer.toPoint2D(stateTime.x());
      Shape shape = new Rectangle2D.Double(point2d.getX(), point2d.getY(), 2, 2);
      graphics.draw(shape);
    }
  }
}
