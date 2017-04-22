// code by jph
package ch.ethz.idsc.owly.glc.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Shape;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.DoubleSummaryStatistics;

import javax.swing.JComponent;
import javax.swing.event.MouseInputAdapter;
import javax.swing.event.MouseInputListener;

import ch.ethz.idsc.owly.glc.core.Node;
import ch.ethz.idsc.owly.glc.core.StateTime;
import ch.ethz.idsc.owly.glc.core.Trajectory;
import ch.ethz.idsc.owly.glc.core.TrajectoryPlanner;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Range;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.sca.Power;

public class GlcComponent {
  // function ignores all but the first and second entry of x
  static Tensor toAffinePoint(Tensor x) {
    return Tensors.of( //
        x.get(0), //
        x.get(1), //
        RealScalar.ONE);
  }

  private Tensor model2pixel;
  private TrajectoryPlanner trajectoryPlanner = null;

  public GlcComponent() {
    model2pixel = Tensors.matrix(new Number[][] { //
        { 60, 0, 300 }, //
        { 0, -60, 300 }, //
        { 0, 0, 1 }, //
    });
    jComponent.addMouseWheelListener(new MouseWheelListener() {
      @Override
      public void mouseWheelMoved(MouseWheelEvent event) {
        int exp = -event.getWheelRotation();
        Scalar factor = Power.of(RealScalar.of(2), exp);
        Tensor scale = DiagonalMatrix.of(Tensors.of(factor, factor, RealScalar.ONE));
        model2pixel = model2pixel.dot(scale);
        jComponent.repaint();
      }
    });
    MouseInputListener mouseInputListener = new MouseInputAdapter() {
      Point down = null;

      @Override
      public void mousePressed(MouseEvent event) {
        down = event.getPoint();
      }

      @Override
      public void mouseDragged(MouseEvent event) {
        Point now = event.getPoint();
        int dx = now.x - down.x;
        int dy = now.y - down.y;
        down = now;
        model2pixel.set(s -> s.add(RealScalar.of(dx)), 0, 2);
        model2pixel.set(s -> s.add(RealScalar.of(dy)), 1, 2);
        jComponent.repaint();
      }
    };
    jComponent.addMouseMotionListener(mouseInputListener);
    jComponent.addMouseListener(mouseInputListener);
  }

  final JComponent jComponent = new JComponent() {
    @Override
    protected void paintComponent(Graphics g) {
      g.setColor(Color.WHITE);
      Dimension dimension = getSize();
      g.fillRect(0, 0, dimension.width, dimension.height);
      // ---
      Graphics2D graphics = (Graphics2D) g;
      {
        graphics.setColor(Color.LIGHT_GRAY);
        graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(-10, 0)), toPoint2D(Tensors.vector(10, 0))));
        graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(0, -10)), toPoint2D(Tensors.vector(0, 10))));
      }
      if (trajectoryPlanner != null) {
        {
          int rgb = 192 + 32;
          graphics.setColor(new Color(rgb, rgb, rgb, 16));
          Tensor scale = trajectoryPlanner.getResolution().map(Scalar::invert);
          for (Tensor x : Range.of(-10, 10))
            for (Tensor y : Range.of(-10, 10)) {
              // Tensor res = scale.pmul(Tensors.of(x, y));
              // graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(-10, res.Get(1).number())), toPoint2D(Tensors.vector(10, res.Get(1).number()))));
              // graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(res.Get(0).number(), -10)), toPoint2D(Tensors.vector(res.Get(0).number(), 10))));
            }
        }
        {
          int rgb = 64;
          graphics.setColor(new Color(rgb, rgb, rgb, 128));
          for (Node node : trajectoryPlanner.getQueue()) {
            Tensor x = node.x;
            Point2D p = toPoint2D(x);
            Shape shape = new Rectangle2D.Double(p.getX() - 1, p.getY() - 1, 3, 3);
            graphics.fill(shape);
          }
        }
        {
          DoubleSummaryStatistics dss = trajectoryPlanner.getNodes().stream() //
              .map(n -> n.cost) //
              .map(Scalar::number) //
              .mapToDouble(Number::doubleValue) //
              .filter(Double::isFinite) //
              .summaryStatistics();
          final double min = dss.getMin();
          final double max = dss.getMax();
          graphics.setColor(Color.BLUE);
          for (Node node : trajectoryPlanner.getNodes()) {
            Scalar cost = node.cost;
            double val = cost.number().doubleValue();
            double interp = (val - min) / (max - min);
            graphics.setColor(new Hue(interp, 1, 1, 1).rgba);
            Point2D p = toPoint2D(node.x);
            {
              Shape shape = new Rectangle2D.Double(p.getX(), p.getY(), 1, 1);
              graphics.fill(shape);
            }
            Node parent = node.parent;
            if (parent != null) {
              Point2D p2 = toPoint2D(parent.x);
              {
                graphics.setColor(new Hue(interp, 1, 1, .1).rgba);
                Shape shape = new Line2D.Double(p.getX(), p.getY(), p2.getX(), p2.getY());
                graphics.draw(shape);
              }
            }
          }
        }
        {
          graphics.setColor(new Color(255, 0, 0, 128));
          Trajectory trajectory = trajectoryPlanner.getPathFromRootToGoal();
          StateTime prev = null;
          for (StateTime stateTime : trajectory) {
            if (prev != null)
              graphics.draw(new Line2D.Double(toPoint2D(prev.x), toPoint2D(stateTime.x)));
            prev = stateTime;
          }
        }
        {
          graphics.setStroke(new BasicStroke(2.0f));
          graphics.setColor(new Color(0, 192, 0, 128));
          Trajectory trajectory = trajectoryPlanner.getDetailedTrajectory();
          StateTime prev = null;
          for (StateTime stateTime : trajectory) {
            if (prev != null)
              graphics.draw(new Line2D.Double(toPoint2D(prev.x), toPoint2D(stateTime.x)));
            prev = stateTime;
          }
          graphics.setStroke(new BasicStroke());
        }
      }
    }
  };

  public void setTrajectoryPlanner(TrajectoryPlanner trajectoryPlanner) {
    this.trajectoryPlanner = trajectoryPlanner;
    jComponent.repaint();
  }

  public Point2D toPoint2D(Tensor x) {
    Tensor point = model2pixel.dot(toAffinePoint(x));
    return new Point2D.Double( //
        point.Get(0).number().doubleValue(), //
        point.Get(1).number().doubleValue());
  }
}
