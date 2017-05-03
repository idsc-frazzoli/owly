// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;

public class GoalLayer extends AbstractLayer {
  GoalLayer(GlcComponent glcComponent) {
    super(glcComponent);
  }

  @Override
  void render(Graphics2D graphics, TrajectoryPlanner trajectoryPlanner) {
    { // show points discovered in the goal region
      TrajectoryRegionQuery trq = trajectoryPlanner.getGoalQuery();
      if (trq instanceof SimpleTrajectoryRegionQuery) {
        SimpleTrajectoryRegionQuery strq = (SimpleTrajectoryRegionQuery) trq;
        graphics.setColor(new Color(224, 168, 0, 224));
        for (StateTime stateTime : strq.getDiscoveredMembers()) {
          Point2D point2d = toPoint2D(stateTime.x());
          graphics.fill(new Ellipse2D.Double(point2d.getX() - 3, point2d.getY() - 3, 5, 5));
        }
      }
    }
  }
}
