// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;

import ch.ethz.idsc.owly.glc.adapter.SimpleTrajectoryRegionQuery;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.opt.ConvexHull;

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
        { // draw convex hull of goal points
          Tensor points = Tensor.of(strq.getDiscoveredMembers().stream().map(StateTime::x).map(x -> x.extract(0, 2)));
          if (0 < points.length()) {
            Tensor hull = ConvexHull.of(points);
            Path2D path2d = new Path2D.Double();
            boolean init = false;
            for (Tensor p : hull) {
              Point2D point2d = toPoint2D(p);
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
          graphics.setColor(new Color(224, 168, 0, 224));
          for (StateTime stateTime : strq.getDiscoveredMembers()) {
            Point2D point2d = toPoint2D(stateTime.x());
            graphics.fill(new Ellipse2D.Double(point2d.getX() - 2.5, point2d.getY() - 2.5, 4, 4));
          }
        }
      }
    }
  }
}
