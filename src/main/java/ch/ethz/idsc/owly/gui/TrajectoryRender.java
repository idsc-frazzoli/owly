// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.Collectors;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.owly.glc.core.GlcNodes;
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
  private TrajectoryPlanner trajectoryPlanner;

  public TrajectoryRender(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    if (Objects.isNull(trajectoryPlanner))
      return;
    // ---
    TrajectoryPlanner trajectoryPlanner = this.trajectoryPlanner;
    Optional<GlcNode> optional = trajectoryPlanner.getBestOrElsePeek();
    if (optional.isPresent()) {
      final GlcNode node = optional.get();
      {// draw detailed trajectory from root to goal/furthestgo
        final List<TrajectorySample> list = trajectoryPlanner.detailedTrajectoryTo(node);
        { // draw control vectors u along trajectory
          int rgb = 64;
          graphics.setColor(new Color(rgb, rgb, rgb, 192));
          for (TrajectorySample trajectorySample : list) {
            Optional<Flow> flow = trajectorySample.getFlow();
            if (flow.isPresent()) {
              Tensor u = flow.get().getU().copy();
              while (u.length() < 2)
                u.append(RealScalar.ZERO);
              graphics.draw( //
                  owlyLayer.toVector( //
                      trajectorySample.stateTime().x(), //
                      u.multiply(U_SCALE) //
                  ));
            }
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
        for (StateTime stateTime : GlcNodes.getPathFromRootTo(node)) {
          Point2D point2d = owlyLayer.toPoint2D(stateTime.x());
          graphics.draw(new Rectangle2D.Double(point2d.getX() - 1, point2d.getY() - 1, 2, 2));
        }
      }
    }
  }

  public void setTrajectoryPlanner(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
  }
}
