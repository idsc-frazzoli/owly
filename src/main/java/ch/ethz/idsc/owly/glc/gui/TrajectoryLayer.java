// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectoryRegionQuery;
import ch.ethz.idsc.owly.math.StateTime;

public class TrajectoryLayer extends AbstractLayer {
  TrajectoryLayer(GlcComponent glcComponent) {
    super(glcComponent);
  }

  @Override
  void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner) {
    {
      graphics.setColor(new Color(255, 0, 0, 128));
      List<StateTime> trajectory = trajectoryPlanner.getPathFromRootToGoal();
      StateTime prev = null;
      for (StateTime stateTime : trajectory) {
        if (prev != null) {
          Point2D p = toPoint2D(prev.x);
          Shape shape = new Rectangle2D.Double(p.getX(), p.getY(), 2, 2);
          graphics.draw(shape);
        }
        prev = stateTime;
      }
    }
    {
      graphics.setStroke(new BasicStroke(2.0f));
      graphics.setColor(new Color(0, 192, 0, 255));
      List<StateTime> trajectory = trajectoryPlanner.getDetailedTrajectory();
      StateTime prev = null;
      for (StateTime stateTime : trajectory) {
        if (prev != null)
          graphics.draw(new Line2D.Double(toPoint2D(prev.x), toPoint2D(stateTime.x)));
        prev = stateTime;
      }
      graphics.setStroke(new BasicStroke());
    }
    {
      TrajectoryRegionQuery trq = trajectoryPlanner.getGoalQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        graphics.setColor(new Color(255, 0, 0, 255));
        for (StateTime st : strq.getDiscoveredMembers()) {
          Point2D p = toPoint2D(st.x);
          Shape shape = new Rectangle2D.Double(p.getX(), p.getY(), 3, 3);
          graphics.fill(shape);
        }
      }
    }
  }
}
