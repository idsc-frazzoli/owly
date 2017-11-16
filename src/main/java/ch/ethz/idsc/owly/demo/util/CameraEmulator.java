// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.Objects;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.se2.Se2Bijection;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class CameraEmulator implements RenderInterface {
  private final int resolution;
  private final Scalar interval;
  private final Supplier<StateTime> supplier;
  private final TrajectoryRegionQuery raytraceQuery;
  // ---
  private BufferedImage bufferedImage;
  private Scalar next;

  /** @param resolution of image in pixels along width and height
   * @param frameRate
   * @param supplier
   * @param raytraceQuery */
  public CameraEmulator( //
      int resolution, Scalar frameRate, //
      Supplier<StateTime> supplier, //
      TrajectoryRegionQuery raytraceQuery) {
    this.resolution = resolution;
    this.interval = frameRate.reciprocal();
    this.supplier = supplier;
    this.raytraceQuery = raytraceQuery;
    bufferedImage = new BufferedImage(resolution, resolution, BufferedImage.TYPE_INT_ARGB);
  }

  /** @param stateTime from where to expose
   * @return */
  public BufferedImage exposure(StateTime stateTime) {
    if (Objects.isNull(bufferedImage) || //
        Objects.isNull(next) || Scalars.lessThan(next, stateTime.time())) {
      next = stateTime.time().add(interval);
      Se2Bijection se2Bijection = new Se2Bijection(stateTime.state());
      TensorUnaryOperator forward = se2Bijection.forward();
      Graphics graphics = bufferedImage.getGraphics();
      graphics.setColor(Color.DARK_GRAY);
      graphics.fillRect(0, 0, resolution, resolution);
      graphics.setColor(new Color(192, 255, 192));
      for (int x = 0; x < bufferedImage.getWidth(); ++x) {
        double xn = x / (double) resolution;
        double dist = 0.6 + 1.5 * xn + xn * xn;
        for (int y = 0; y < bufferedImage.getHeight(); ++y) {
          Tensor probe = forward.apply(Tensors.vector(dist, (y - resolution / 2) * 0.02 * dist));
          if (raytraceQuery.isMember(new StateTime(probe, stateTime.time()))) {
            // Point2D point2D = geometricLayer.toPoint2D(probe);
            // graphics.fillRect((int) point2D.getX(), (int) point2D.getY(), 2, 2);
          } else
            graphics.fillRect(resolution - y - 1, resolution - x - 1, 1, 1);
        }
      }
    }
    return bufferedImage;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    BufferedImage bufferedImage = exposure(supplier.get()); // TODO exposure should not happen inside drawing
    int SCREEN = resolution * 2;
    int OFFSET = 20;
    graphics.setColor(Color.GREEN);
    graphics.drawRect(0, OFFSET, SCREEN + 1, SCREEN + 1);
    graphics.drawImage(bufferedImage, 1, OFFSET + 1, SCREEN, SCREEN, null);
  }
}
