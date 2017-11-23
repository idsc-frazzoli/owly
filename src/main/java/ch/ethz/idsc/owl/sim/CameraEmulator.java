// code by jph
package ch.ethz.idsc.owl.sim;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.Objects;
import java.util.function.Supplier;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.math.map.Se2Bijection;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;

// TODO frame rate should be handled outside this class!
public class CameraEmulator implements RenderInterface {
  private static final Color CLEAR_COLOR = new Color(192, 255, 192);
  // ---
  private final int resolution;
  private final Scalar interval;
  private final Supplier<StateTime> supplier;
  private final TrajectoryRegionQuery raytraceQuery;
  // ---
  private BufferedImage bufferedImage;
  private final Tensor localPoints = Tensors.empty(); // TODO unmodifiable
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
    for (Tensor _xn : Subdivide.of(0, 1, resolution - 1)) {
      double xn = _xn.Get().number().doubleValue();
      double dist = 0.6 + 1.5 * xn + xn * xn;
      for (Tensor _yn : Subdivide.of(-0.5, 0.5, resolution - 1)) {
        double y = _yn.Get().number().doubleValue();
        Tensor probe = Tensors.vector(dist, y * dist);
        localPoints.append(probe);
      }
    }
    GlobalAssert.that(localPoints.length() == resolution * resolution);
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
      graphics.setColor(CLEAR_COLOR);
      int x = 0;
      int y = 0;
      for (Tensor probe : localPoints) {
        if (!raytraceQuery.isMember(new StateTime(forward.apply(probe), stateTime.time())))
          graphics.fillRect(resolution - y - 1, resolution - x - 1, 1, 1);
        ++y;
        if (y == resolution) {
          ++x;
          y = 0;
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
