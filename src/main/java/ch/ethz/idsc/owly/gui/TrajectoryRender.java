// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

class TrajectoryRender implements AbstractRender {
  public static Scalar U_SCALE = RealScalar.of(.33);
  // ---
  private final TrajectoryPlanner trajectoryPlanner;

  TrajectoryRender(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // draw detailed trajectory from root to goal
      GlcNode best = trajectoryPlanner.getBest();
      if (best == null)
        best = trajectoryPlanner.peek();
      // if (best != null)
      final List<TrajectorySample> list = trajectoryPlanner.detailedTrajectoryTo(best);
      { // draw control vectors u along trajectory
        int rgb = 64;
        graphics.setColor(new Color(rgb, rgb, rgb, 192));
        for (TrajectorySample trajectorySample : list)
          if (trajectorySample.hasU()) {
            Tensor u = trajectorySample.getU().copy();
            while (u.length() < 2)
              u.append(RealScalar.ZERO);
            graphics.draw( //
                owlyLayer.toVector( //
                    trajectorySample.stateTime().x(), //
                    u.multiply(U_SCALE) //
                ));
          }
      }
      { // draw trajectory as thick green line with white background
        Path2D path2d = owlyLayer.toPath2D(list.stream().map(TrajectorySample::stateTime).collect(Collectors.toList()));
        graphics.setStroke(new BasicStroke(5.0f));
        graphics.setColor(new Color(255, 255, 255, 128));
        graphics.draw(path2d);
        graphics.setStroke(new BasicStroke(2.0f));
        graphics.setColor(new Color(0, 192, 0, 192));
        graphics.draw(path2d);
        graphics.setStroke(new BasicStroke());
      }
    }
    { // draw boxes at nodes in path from root to goal
      graphics.setColor(new Color(255, 0, 0, 128));
      for (StateTime stateTime : trajectoryPlanner.getPathFromRootToGoal()) {
        Point2D point2d = owlyLayer.toPoint2D(stateTime.x());
        graphics.draw(new Rectangle2D.Double(point2d.getX() - 1, point2d.getY() - 1, 2, 2));
      }
    }
  }
}
