// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;

import ch.ethz.idsc.owly.math.state.StateTime;

class ObstacleLayer extends AbstractLayer {
  Collection<StateTime> collection;

  ObstacleLayer(OwlyComponent glcComponent) {
    super(glcComponent);
  }

  void setDiscoveredMembers(Collection<StateTime> collection) {
    this.collection = collection;
  }

  @Override
  void render(Graphics2D graphics) {
    graphics.setColor(new Color(0, 0, 0, 128));
    for (StateTime stateTime : collection) {
      Point2D point2d = toPoint2D(stateTime.x());
      Shape shape = new Rectangle2D.Double(point2d.getX(), point2d.getY(), 2, 2);
      graphics.draw(shape);
    }
  }
}
