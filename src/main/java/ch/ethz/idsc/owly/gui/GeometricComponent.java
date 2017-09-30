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
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.CopyOnWriteArrayList;

import javax.swing.JComponent;
import javax.swing.event.MouseInputAdapter;
import javax.swing.event.MouseInputListener;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Join;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.LinearSolve;
import ch.ethz.idsc.tensor.sca.Power;
import ch.ethz.idsc.tensor.sca.Round;

public final class GeometricComponent {
  private static final double WHEEL_ANGLE = Math.PI / 10;
  private static final int BUTTON_DRAG = 3;
  private static final Tensor MODEL2PIXEL_INITIAL = Tensors.matrix(new Number[][] { //
      { 60, 0, 300 }, //
      { 0, -60, 300 }, //
      { 0, 0, 1 }, //
  }).unmodifiable();
  /***************************************************/
  /** public access to final JComponent: attach mouse listeners, get/set properties, ... */
  public final JComponent jComponent = new JComponent() {
    @Override
    protected void paintComponent(Graphics graphics) {
      render((Graphics2D) graphics, getSize());
      { // display frame rate only when rendering in component
        long period = System.nanoTime() - lastRepaint;
        lastRepaint = System.nanoTime();
        graphics.setColor(Color.LIGHT_GRAY);
        graphics.drawString(String.format("%4.1f Hz", 1.0e9 / period), 0, 10);
      }
    }
  };
  // 3x3 affine matrix that maps model to pixel coordinates
  private Tensor model2pixel;
  private Tensor mouseLocation = Array.zeros(2);
  private final List<RenderInterface> renderBackground = new CopyOnWriteArrayList<>();
  private final List<RenderInterface> renderInterfaces = new CopyOnWriteArrayList<>();
  private long lastRepaint = System.nanoTime();
  private int mouseWheel = 0;

  public GeometricComponent() {
    reset_model2pixel();
    jComponent.addMouseWheelListener(event -> {
      final int delta = -event.getWheelRotation(); // either 1 or -1
      final int mods = event.getModifiersEx();
      final int mask = MouseWheelEvent.CTRL_DOWN_MASK; // 128 = 2^7
      if ((mods & mask) == 0) { // ctrl pressed?
        mouseWheel += delta;
      } else {
        Scalar factor = Power.of(RealScalar.of(2), delta);
        Tensor scale = DiagonalMatrix.of(Tensors.of(factor, factor, RealScalar.ONE));
        Tensor shift = Tensors.vector(event.getX(), event.getY());
        shift = shift.subtract(shift.multiply(factor));
        scale.set(shift.Get(0), 0, 2);
        scale.set(shift.Get(1), 1, 2);
        model2pixel = scale.dot(model2pixel);
      }
      jComponent.repaint();
    });
    {
      MouseInputListener mouseInputListener = new MouseInputAdapter() {
        Point down = null;

        @Override
        public void mouseMoved(MouseEvent mouseEvent) {
          mouseLocation = toModel(mouseEvent.getPoint());
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
      MouseListener mouseListener = new MouseAdapter() {
        @Override
        public void mousePressed(MouseEvent mouseEvent) {
          Tensor location = toModel(mouseEvent.getPoint());
          System.out.println(location.map(Round._3) + ",");
        }
      };
      jComponent.addMouseListener(mouseListener);
    }
  }

  public void setRenderInterfaces(Collection<RenderInterface> collection) {
    renderInterfaces.clear(); // TODO background
    renderInterfaces.addAll(collection);
  }

  public void addRenderInterface(RenderInterface renderInterface) {
    renderInterfaces.add(renderInterface);
  }

  /** @return {px, py, angle} in model space */
  public Tensor getMouseGoal() {
    // TODO design
    return createLayer().getMouseSe2State();
  }

  public void addRenderInterfaceBackground(RenderInterface renderInterface) {
    renderBackground.add(renderInterface);
  }

  /***************************************************/
  /** @param vector */
  void setOffset(Tensor vector) {
    model2pixel.set(vector.Get(0), 0, 2);
    model2pixel.set(vector.Get(1), 1, 2);
  }

  void reset_model2pixel() {
    model2pixel = MODEL2PIXEL_INITIAL.copy();
  }

  void render(Graphics2D graphics, Dimension dimension) {
    graphics.setColor(Color.WHITE);
    graphics.fillRect(0, 0, dimension.width, dimension.height);
    // ---
    renderBackground.forEach(renderInterface -> renderInterface.render(createLayer(), graphics));
    renderInterfaces.forEach(renderInterface -> renderInterface.render(createLayer(), graphics));
  }

  /***************************************************/
  private GeometricLayer createLayer() {
    return new GeometricLayer( //
        model2pixel, //
        Join.of(mouseLocation, Tensors.of(RealScalar.of(mouseWheel * WHEEL_ANGLE))));
  }

  /** transforms point in pixel space to coordinates of model space
   * inverse of function model2Point2D(...)
   * 
   * @param point
   * @return tensor of length 2 */
  private Tensor toModel(Point point) {
    return LinearSolve.of(model2pixel, Tensors.vector(point.x, point.y, 1)).extract(0, 2);
  }
}
