// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

import javax.swing.JComponent;

import ch.ethz.idsc.owly.glc.core.Node;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class GlcComponent {
  public static Tensor toAffineVector(Tensor x) {
    return Tensors.of(x.get(0), x.get(1), RealScalar.ONE);
  }

  public Point2D toPoint(Tensor xx) {
    Tensor x = rep.dot(toAffineVector(xx));
    return new Point2D.Double(x.Get(0).number().doubleValue(), x.Get(1).number().doubleValue());
  }

  Tensor rep;

  public GlcComponent() {
    rep = Tensors.matrixInt(new int[][] { //
        { 10, 0, 200 }, //
        { 0, -10, 200 }, //
        { 0, 0, 1 }, //
    });
  }

  TrajectoryPlanner trajectoryPlanner = null;
  JComponent jComponent = new JComponent() {
    @Override
    protected void paintComponent(Graphics g) {
      Graphics2D graphics = (Graphics2D) g;
      if (trajectoryPlanner != null) {
        graphics.setColor(Color.BLACK);
        for (Node node : trajectoryPlanner.queue) {
          Tensor x = node.x;
          Point2D p = toPoint(x);
          graphics.drawLine((int) p.getX(), (int) p.getY(), (int) p.getX() + 1, (int) p.getY() + 1);
        }
        {
          graphics.setColor(Color.RED);
          Trajectory trajectory = trajectoryPlanner.getPathFromGoalToRoot();
          // Shape asd = new Line2D.Double(0, 0, 0, 0);
          // graphics.draw
          StateTime prev = null;
          for (StateTime stateTime : trajectory) {
            // Point2D p = ;
            if (prev != null) {
              Shape asd = new Line2D.Double(toPoint(prev.tensor), toPoint(stateTime.tensor));
              graphics.draw(asd);
            }
            prev = stateTime;
          }
        }
        // trajectoryPlanner.
      }
    }
  };

  public void setTrajectoryPlanner(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
    jComponent.repaint();
  }
}
