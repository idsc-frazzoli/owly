// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.Objects;

import ch.ethz.idsc.owly.math.state.StateTime;

public class ObstacleRender implements RenderInterface {
  private Collection<StateTime> collection;

  public ObstacleRender(Collection<StateTime> collection) {
    this.collection = collection;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    if (Objects.isNull(collection))
      return;
    // ---
    graphics.setColor(new Color(0, 0, 0, 128));
    for (StateTime stateTime : collection) {
      Point2D point2d = owlyLayer.toPoint2D(stateTime.state());
      Shape shape = new Rectangle2D.Double(point2d.getX(), point2d.getY(), 2, 2);
      graphics.draw(shape);
    }
  }

  public void setCollection(Collection<StateTime> collection) {
    this.collection = collection;
  }
}
