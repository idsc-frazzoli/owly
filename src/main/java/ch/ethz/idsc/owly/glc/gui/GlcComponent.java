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
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.JComponent;

import ch.ethz.idsc.owly.glc.core.Node;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
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
        { 60, 0, 300 }, //
        { 0, -60, 300 }, //
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
        {
          DoubleSummaryStatistics dss = trajectoryPlanner.getQueue().stream().mapToDouble(n -> n.cost.number().doubleValue()).summaryStatistics();
          dss.getMin();
          dss.getMax();
        }
        {
          graphics.setColor(new Color(128, 128, 128, 128));
          for (Node node : trajectoryPlanner.getQueue()) {
            Tensor x = node.x;
            Point2D p = toPoint(x);
            Shape shape = new Rectangle2D.Double(p.getX(), p.getY(), 2, 2);
            graphics.fill(shape);
          }
        }
        {
          DoubleSummaryStatistics dss = trajectoryPlanner.getDomains().values().stream() //
              .map(n -> n.cost) //
              .map(Scalar::number) //
              .mapToDouble(Number::doubleValue) //
              .filter(Double::isFinite) //
              .summaryStatistics();
          // System.out.println(dss);
          final double min = dss.getMin();
          final double max = dss.getMax();
          Map<Tensor, Node> map = trajectoryPlanner.getDomains();
          graphics.setColor(Color.BLUE);
          for (Entry<Tensor, Node> entry : map.entrySet()) {
            // Tensor key = entry.getKey();
            // Domain domain = entry.getValue();
            Node node = entry.getValue();
            // if (node != null)
            {
              Scalar cost = node.cost;
              double val = cost.number().doubleValue();
              double interp = (val - min) / (max - min);
              // System.out.println(key);
              Hue hue = new Hue(interp, 1, 1, 1);
              graphics.setColor(hue.rgba);
              Tensor x = node.x;
              Point2D p = toPoint(x);
              Shape shape = new Rectangle2D.Double(p.getX(), p.getY(), 1, 1);
              graphics.fill(shape);
            }
          }
        }
        {
          graphics.setColor(new Color(255, 0, 0, 128));
          Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
          StateTime prev = null;
          for (StateTime stateTime : trajectory) {
            if (prev != null)
              graphics.draw(new Line2D.Double(toPoint(prev.x), toPoint(stateTime.x)));
            prev = stateTime;
          }
        }
        {
          graphics.setColor(new Color(0, 192, 0, 128));
          Trajectory trajectory = trajectoryPlanner.getDetailedTrajectory();
          StateTime prev = null;
          for (StateTime stateTime : trajectory) {
            if (prev != null)
              graphics.draw(new Line2D.Double(toPoint(prev.x), toPoint(stateTime.x)));
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
