// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.owly.glc.core.TrajectorySample;
import ch.ethz.idsc.owly.math.flow.Flow;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

public class TrajectoryRender implements RenderInterface {
  public static Scalar U_SCALE = RealScalar.of(.33);
  // ---
  private List<TrajectorySample> trajectory = new ArrayList<>();
  private Color trajectoryColor = new Color(0, 192, 0, 192);

  public TrajectoryRender(TrajectoryPlanner trajectoryPlanner) {
    // TODO design not elegant!
    if (Objects.nonNull(trajectoryPlanner)) {
      Optional<GlcNode> optional = trajectoryPlanner.getFinalGoalNode();
      if (optional.isPresent()) {
        final GlcNode node = optional.get();
        { // draw detailed trajectory from root to goal/furthestgo
          setTrajectory(trajectoryPlanner.detailedTrajectoryTo(node));
        }
      }
    }
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    { // draw detailed trajectory from root to goal/furthestgo
      final List<TrajectorySample> list = trajectory;
      { // draw control vectors u along trajectory
        int rgb = 64;
        graphics.setColor(new Color(rgb, rgb, rgb, 192));
        for (TrajectorySample trajectorySample : list) {
          Optional<Flow> flow = trajectorySample.getFlow();
          if (flow.isPresent()) {
            Tensor uscaled = flow.get().getU().multiply(U_SCALE);
            while (uscaled.length() < 2)
              uscaled.append(RealScalar.ZERO);
            graphics.draw(owlyLayer.toVector(trajectorySample.stateTime().state(), uscaled));
          }
        }
      }
      { // draw trajectory as thick green line with white background
        Path2D path2d = owlyLayer.toPath2D(list.stream().map(TrajectorySample::stateTime).collect(Collectors.toList()));
        graphics.setStroke(new BasicStroke(5.0f));
        graphics.setColor(new Color(255, 255, 255, 128));
        graphics.draw(path2d);
        graphics.setStroke(new BasicStroke(2.0f));
        graphics.setColor(trajectoryColor);
        graphics.draw(path2d);
        graphics.setStroke(new BasicStroke());
      }
    }
    { // draw boxes at nodes in path from root to goal
      graphics.setColor(new Color(255, 0, 0, 96));
      trajectory.stream().map(TrajectorySample::stateTime).map(StateTime::state).forEach(state -> {
        Point2D point2d = owlyLayer.toPoint2D(state);
        graphics.draw(new Rectangle2D.Double(point2d.getX() - 1, point2d.getY() - 1, 2, 2));
      });
    }
  }

  public void setTrajectory(List<TrajectorySample> trajectory) {
    this.trajectory = trajectory;
  }

  public void setColor(Color newTrajectoryColor) {
    trajectoryColor = newTrajectoryColor;
  }
}
