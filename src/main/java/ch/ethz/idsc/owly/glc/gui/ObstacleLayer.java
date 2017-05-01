// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.state.StateTime;

class ObstacleLayer extends AbstractLayer {
  ObstacleLayer(GlcComponent glcComponent) {
    super(glcComponent);
  }

  @Override
  void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner) {
    TrajectoryRegionQuery trq = trajectoryPlanner.getObstacleQuery();
    if (trq instanceof SimpleTrajectoryRegionQuery) {
      SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
      graphics.setColor(new Color(0, 0, 0, 64));
      for (StateTime st : strq.getDiscoveredMembers()) {
        Point2D p = toPoint2D(st.x);
        Shape shape = new Rectangle2D.Double(p.getX(), p.getY(), 2, 2);
        graphics.draw(shape);
      }
    }
  }
}
