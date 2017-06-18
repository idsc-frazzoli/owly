// code by jph
package ch.ethz.idsc.owly.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

import javax.swing.JComponent;
import javax.swing.event.MouseInputAdapter;
import javax.swing.event.MouseInputListener;

import ch.ethz.idsc.tensor.DecimalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.LinearSolve;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Round;

class OwlyComponent {
  static final Tensor MODEL2PIXEL_INITIAL = Tensors.matrix(new Number[][] { //
      { 60, 0, 300 }, //
      { 0, -60, 300 }, //
      { 0, 0, 1 }, //
  }).unmodifiable();

  // function ignores all but the first and second entry of x
  private static Tensor toAffinePoint(Tensor x) {
    return Tensors.of( //
        x.get(0), //
        x.get(1), //
        RealScalar.ONE);
  }

  Tensor model2pixel;
  final OwlyLayer abstractLayer = new OwlyLayer(this);
  RenderElements renderElements;

  public OwlyComponent() {
    reset_model2pixel();
    jComponent.addMouseWheelListener(new MouseWheelListener() {
      @Override
      public void mouseWheelMoved(MouseWheelEvent event) {
        int exp = -event.getWheelRotation(); // either 1 or -1
        Scalar factor = Power.of(RealScalar.of(2), exp);
        Tensor scale = DiagonalMatrix.of(Tensors.of(factor, factor, RealScalar.ONE));
        model2pixel = model2pixel.dot(scale);
        jComponent.repaint();
      }
    });
    {
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
    {
      MouseListener mouseListener = new MouseAdapter() {
        @Override
        public void mouseClicked(MouseEvent mouseEvent) {
          // System.out.println("model2pixel=");
          // System.out.println(Pretty.of(model2pixel));
        }

        @Override
        public void mousePressed(MouseEvent mouseEvent) {
          Tensor location = toTensor(mouseEvent.getPoint());
          location = location.extract(0, 2);
          // info for user to design obstacles or check distances
          System.out.println(location.map(Round.toMultipleOf(DecimalScalar.of(0.001))) + ",");
        }
      };
      jComponent.addMouseListener(mouseListener);
    }
  }

  void reset_model2pixel() {
    model2pixel = MODEL2PIXEL_INITIAL.copy();
  }

  final JComponent jComponent = new JComponent() {
    @Override
    protected void paintComponent(Graphics g) {
      render((Graphics2D) g, getSize());
    }
  };

  void render(Graphics2D graphics, Dimension dimension) {
    graphics.setColor(Color.WHITE);
    graphics.fillRect(0, 0, dimension.width, dimension.height);
    {
      graphics.setColor(Color.LIGHT_GRAY);
      graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(-10, 1)), toPoint2D(Tensors.vector(10, 1))));
      graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(1, -10)), toPoint2D(Tensors.vector(1, 10))));
    }
    {
      graphics.setColor(Color.GRAY);
      graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(-10, 0)), toPoint2D(Tensors.vector(10, 0))));
      graphics.draw(new Line2D.Double(toPoint2D(Tensors.vector(0, -10)), toPoint2D(Tensors.vector(0, 10))));
    }
    if (renderElements != null) {
      renderElements.list.forEach(ar -> ar.render(abstractLayer, graphics));
    }
  }

  public Point2D toPoint2D(Tensor x) {
    Tensor point = model2pixel.dot(toAffinePoint(x));
    return new Point2D.Double( //
        point.Get(0).number().doubleValue(), //
        point.Get(1).number().doubleValue());
  }

  public Tensor toTensor(Point point) {
    return LinearSolve.of( //
        model2pixel, //
        toAffinePoint(Tensors.vector(point.x, point.y)));
  }
}
