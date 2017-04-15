// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.DoubleSummaryStatistics;

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
        { 40, 0, 300 }, //
        { 0, -40, 300 }, //
        { 0, 0, 1 }, //
    });
  }

  TrajectoryPlanner trajectoryPlanner = null;
  JComponent jComponent = new JComponent() {
    @Override
    protected void paintComponent(Graphics g) {
      g.setColor(Color.WHITE);
      Dimension dimension = getSize();
      g.fillRect(0, 0, dimension.width, dimension.height);
      // ---
      Graphics2D graphics = (Graphics2D) g;
      {
        graphics.setColor(Color.LIGHT_GRAY);
        graphics.draw(new Line2D.Double(toPoint(Tensors.vector(-10, 0)), toPoint(Tensors.vector(10, 0))));
        graphics.draw(new Line2D.Double(toPoint(Tensors.vector(0, -10)), toPoint(Tensors.vector(0, 10))));
      }
      if (trajectoryPlanner != null) {
        graphics.setColor(Color.BLACK);
        DoubleSummaryStatistics dss = trajectoryPlanner.queue.stream().mapToDouble(n->n.cost.number().doubleValue()).summaryStatistics();
        System.out.println(dss);
        dss.getMin();
        dss.getMax();
        for (Node node : trajectoryPlanner.queue) {
          Tensor x = node.x;
          Point2D p = toPoint(x);
          Shape shape = new Rectangle2D.Double(p.getX(), p.getY(), 1, 1);
//          graphics.drawLine((int) p.getX(), (int) p.getY(), (int) p.getX() + 1, (int) p.getY() + 1);
          graphics.fill(shape);
        }
        {
          graphics.setColor(new Color(255, 0, 0, 128));
          Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
          StateTime prev = null;
          for (StateTime stateTime : trajectory) {
            if (prev != null)
              graphics.draw(new Line2D.Double(toPoint(prev.tensor), toPoint(stateTime.tensor)));
            prev = stateTime;
          }
        }
        {
          graphics.setColor(new Color(0, 192, 0, 128));
          Trajectory trajectory = trajectoryPlanner.getDetailed();
          StateTime prev = null;
          for (StateTime stateTime : trajectory) {
            if (prev != null)
              graphics.draw(new Line2D.Double(toPoint(prev.tensor), toPoint(stateTime.tensor)));
            prev = stateTime;
          }
        }
      }
    }
  };

  public void setTrajectoryPlanner(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
    jComponent.repaint();
  }
}
