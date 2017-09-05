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
import java.util.Objects;

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

public class OwlyComponent {
  private static final int BUTTON_DRAG = 3;
  private static final Tensor MODEL2PIXEL_INITIAL = Tensors.matrix(new Number[][] { //
      { 60, 0, 300 }, //
      { 0, -60, 300 }, //
      { 0, 0, 1 }, //
  }).unmodifiable();

  // function ignores all but the first and second entry of x
  private static Tensor toAffinePoint(Tensor x) {
    return Tensors.of(x.get(0), x.get(1), RealScalar.ONE);
  }

  /***************************************************/
  // 3x3 affine matrix that maps model to pixel coordinates
  private Tensor model2pixel;
  private final OwlyLayer owlyLayer = new OwlyLayer() {
    @Override
    public Point2D toPoint2D_function(Tensor tensor) {
      return model2Point2D(tensor);
    }

    @Override
    public Tensor model2pixel() {
      return model2pixel.unmodifiable();
    }
  };
  /** public access to final JComponent: attach mouse listeners, get/set properties, ... */
  public final JComponent jComponent = new JComponent() {
    @Override
    protected void paintComponent(Graphics graphics) {
      render((Graphics2D) graphics, getSize());
    }
  };
  private RenderElements renderBackground = new RenderElements();
  public RenderElements renderElements; // TODO use setter function
  private long lastRepaint = System.nanoTime();

  public OwlyComponent() {
    reset_model2pixel();
    jComponent.addMouseWheelListener(new MouseWheelListener() {
      @Override
      public void mouseWheelMoved(MouseWheelEvent event) {
        final int delta = -event.getWheelRotation(); // either 1 or -1
        int mods = event.getModifiersEx();
        if ((mods & 128) == 0) { // ctrl pressed?
          owlyLayer.incrementMouseWheel(delta);
        } else {
          Scalar factor = Power.of(RealScalar.of(2), delta);
          Tensor scale = DiagonalMatrix.of(Tensors.of(factor, factor, RealScalar.ONE));
          model2pixel = model2pixel.dot(scale);
        }
        jComponent.repaint();
      }
    });
    {
      MouseInputListener mouseInputListener = new MouseInputAdapter() {
        Point down = null;

        @Override
        public void mouseMoved(MouseEvent mouseEvent) {
          owlyLayer.setMouseLocation(toModel(mouseEvent.getPoint()));
        }

        @Override
        public void mousePressed(MouseEvent mouseEvent) {
          if (mouseEvent.getButton() == BUTTON_DRAG)
            down = mouseEvent.getPoint();
        }

        @Override
        public void mouseDragged(MouseEvent mouseEvent) {
          if (Objects.nonNull(down)) {
            Point now = mouseEvent.getPoint();
            int dx = now.x - down.x;
            int dy = now.y - down.y;
            down = now;
            model2pixel.set(s -> s.add(RealScalar.of(dx)), 0, 2);
            model2pixel.set(s -> s.add(RealScalar.of(dy)), 1, 2);
            jComponent.repaint();
          }
        }

        @Override
        public void mouseReleased(MouseEvent mouseEvent) {
          down = null;
        }
      };
      jComponent.addMouseMotionListener(mouseInputListener);
      jComponent.addMouseListener(mouseInputListener);
    }
    {
      // @SuppressWarnings("unused")
      MouseListener mouseListener = new MouseAdapter() {
        @Override
        public void mousePressed(MouseEvent mouseEvent) {
          Tensor location = toModel(mouseEvent.getPoint());
          System.out.println(location.map(Round.toMultipleOf(DecimalScalar.of(0.001))) + ",");
        }
      };
      jComponent.addMouseListener(mouseListener);
    }
  }

  void reset_model2pixel() {
    model2pixel = MODEL2PIXEL_INITIAL.copy();
  }

  void render(Graphics2D graphics, Dimension dimension) {
    graphics.setColor(Color.WHITE);
    graphics.fillRect(0, 0, dimension.width, dimension.height);
    // ---
    renderBackground.list.forEach(renderInterface -> renderInterface.render(owlyLayer, graphics));
    { // display frame rate
      long period = System.nanoTime() - lastRepaint;
      lastRepaint = System.nanoTime();
      graphics.setColor(Color.GRAY);
      // graphics.drawString(String.format("%4.1f Hz \u0f5c", 1.0e9 / period), 0, 10);
    }
    {
      graphics.setColor(Color.LIGHT_GRAY);
      graphics.draw(new Line2D.Double(model2Point2D(Tensors.vector(-10, 1)), model2Point2D(Tensors.vector(10, 1))));
      graphics.draw(new Line2D.Double(model2Point2D(Tensors.vector(1, -10)), model2Point2D(Tensors.vector(1, 10))));
    }
    {
      graphics.setColor(Color.GRAY);
      graphics.draw(new Line2D.Double(model2Point2D(Tensors.vector(-10, 0)), model2Point2D(Tensors.vector(10, 0))));
      graphics.draw(new Line2D.Double(model2Point2D(Tensors.vector(0, -10)), model2Point2D(Tensors.vector(0, 10))));
    }
    if (Objects.nonNull(renderElements)) {
      renderElements.list.forEach(renderInterface -> renderInterface.render(owlyLayer, graphics));
    }
  }

  /* package */ Point2D model2Point2D(Tensor x) {
    Tensor point = model2pixel.dot(toAffinePoint(x));
    return new Point2D.Double( //
        point.Get(0).number().doubleValue(), //
        point.Get(1).number().doubleValue());
  }

  /** transforms point in pixel space to coordinates of model space
   * inverse of function {@link OwlyComponent#toPoint2D(Tensor)}
   * 
   * @param point
   * @return tensor of length 2 */
  public Tensor toModel(Point point) {
    return LinearSolve.of(model2pixel, toAffinePoint(Tensors.vector(point.x, point.y))).extract(0, 2);
  }

  /** @param vector */
  public void setOffset(Tensor vector) {
    model2pixel.set(vector.Get(0), 0, 2);
    model2pixel.set(vector.Get(1), 1, 2);
  }

  /** @return {px, py, angle} in model space */
  public Tensor getMouseGoal() {
    return owlyLayer.getMouseSe2State();
  }

  public void addDrawable(RenderInterface renderInterface) {
    renderBackground.list.add(renderInterface);
  }
}
