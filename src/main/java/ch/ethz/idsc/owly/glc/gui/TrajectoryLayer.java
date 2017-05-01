// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;

public class TrajectoryLayer extends AbstractLayer {
  TrajectoryLayer(GlcComponent glcComponent) {
    super(glcComponent);
  }

  @Override
  void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner) {
    { // draw detailed trajectory from root to goal
      Path2D path2d = toPath2D(trajectoryPlanner.detailedTrajectoryToGoal());
      graphics.setStroke(new BasicStroke(5.0f));
      graphics.setColor(new Color(255, 255, 255, 128));
      graphics.draw(path2d);
      graphics.setStroke(new BasicStroke(2.0f));
      graphics.setColor(new Color(0, 192, 0, 192));
      graphics.draw(path2d);
      graphics.setStroke(new BasicStroke());
    }
    { // draw boxes at nodes in path from root to goal
      graphics.setColor(new Color(255, 0, 0, 128));
      for (StateTime stateTime : trajectoryPlanner.getPathFromRootToGoal()) {
        Point2D point2d = toPoint2D(stateTime.x());
        graphics.draw(new Rectangle2D.Double(point2d.getX() - 1, point2d.getY() - 1, 2, 2));
      }
    }
    { // show points discovered in the goal region
      TrajectoryRegionQuery trq = trajectoryPlanner.getGoalQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        graphics.setColor(new Color(255, 0, 0, 255));
        for (StateTime stateTime : strq.getDiscoveredMembers()) {
          Point2D point2d = toPoint2D(stateTime.x());
          graphics.fill(new Rectangle2D.Double(point2d.getX() - 1, point2d.getY() - 1, 3, 3));
        }
      }
    }
  }
}
