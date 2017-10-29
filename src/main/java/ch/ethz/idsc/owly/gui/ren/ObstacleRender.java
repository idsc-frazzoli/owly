// code by jph
package ch.ethz.idsc.owly.gui.ren;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Objects;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.StateTimeCollector;

/** coordinates of detected obstacles are rendered as gray squares */
public class ObstacleRender implements RenderInterface {
  private Collection<StateTime> collection;

  public ObstacleRender(Collection<StateTime> collection) {
    this.collection = collection;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    if (Objects.isNull(collection))
      return;
    // ---
    // fillRect is 4x faster than drawRect
    graphics.setColor(new Color(0, 0, 0, 128));
    for (StateTime stateTime : collection) {
      Point2D point2d = geometricLayer.toPoint2D(stateTime.state());
      graphics.fillRect((int) point2d.getX(), (int) point2d.getY(), 2, 2);
    }
  }

  public void fromStateTimeCollector(Object object) {
    collection = object instanceof StateTimeCollector //
        ? new HashSet<>(((StateTimeCollector) object).getMembers()) //
        : Collections.emptySet();
  }
}
