// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.RnRaytracer;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.se2.Se2Bijection;
import ch.ethz.idsc.owly.math.se2.Se2Utils;
import ch.ethz.idsc.owly.math.state.EpisodeIntegrator;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.sca.N;

/** several magic constants are hard-coded in the implementation.
 * that means, the functionality does not apply to all examples universally. */
public abstract class Se2Entity extends AbstractEntity {
  public static final Tensor FALLBACK_CONTROL = N.DOUBLE.of(Array.zeros(3)).unmodifiable();
  public final Collection<CostFunction> extraCosts = new LinkedList<>();
  public TrajectoryRegionQuery obstacleQuery = null;
  public TrajectoryRegionQuery raytraceQuery = null;

  public Se2Entity(EpisodeIntegrator episodeIntegrator) {
    super(episodeIntegrator);
  }

  @Override
  protected final Tensor fallbackControl() {
    return FALLBACK_CONTROL;
  }

  private boolean obstacleQuery_isDisjoint(StateTime stateTime) {
    return Objects.isNull(obstacleQuery) || !obstacleQuery.isMember(stateTime);
  }

  protected abstract Tensor eta();

  protected abstract Tensor shape();

  @Override
  public final void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    { // indicate current position
      final StateTime stateTime = getStateTimeNow();
      Color color = obstacleQuery_isDisjoint(stateTime) //
          ? new Color(64, 64, 64, 128)
          : new Color(255, 64, 64, 128);
      geometricLayer.pushMatrix(Se2Utils.toSE2Matrix(stateTime.state()));
      graphics.setColor(color);
      graphics.fill(geometricLayer.toPath2D(shape()));
      geometricLayer.popMatrix();
    }
    { // indicate position delay[s] into the future
      Tensor state = getEstimatedLocationAt(delayHint());
      Point2D point = geometricLayer.toPoint2D(state);
      graphics.setColor(new Color(255, 128, 64, 192));
      graphics.fill(new Rectangle2D.Double(point.getX() - 2, point.getY() - 2, 5, 5));
    }
    if (Objects.nonNull(raytraceQuery)) {
      StateTime stateTime = getStateTimeNow();
      RnRaytracer lidarRaytrace = new RnRaytracer(raytraceQuery);
      Tensor xya = stateTime.state();
      graphics.setColor(new Color(255, 0, 0, 32));
      Tensor range = Array.zeros(129);
      int index = 0;
      for (Tensor angle : Subdivide.of(+Math.PI / 2, -Math.PI / 2, range.length() - 1)) {
        Tensor origin = xya.extract(0, 2).add(AngleVector.of(xya.Get(2)).multiply(RealScalar.of(.1)));
        Optional<StateTime> optional = lidarRaytrace.firstMember( //
            new StateTime(origin, RealScalar.ZERO), AngleVector.of(xya.Get(2).add(angle)));
        if (optional.isPresent()) {
          Scalar tof = optional.get().time().multiply(RealScalar.of(10));
          range.set(tof, index);
          if (index % 8 == 0) {
            Tensor collision = optional.get().state();
            Shape line = geometricLayer.toPath2D(Tensors.of(origin, collision));
            graphics.draw(line);
          }
        }
        ++index;
      }
      graphics.setColor(new Color(224, 224, 255, 128));
      graphics.fillRect(0, 200, range.length(), 100);
      graphics.setColor(Color.RED);
      for (index = 0; index < range.length(); ++index)
        graphics.fillRect(index, 300 - range.Get(index).number().intValue(), 1, 2);
    }
    { // draw mouse
      Color color = new Color(0, 128, 255, 192);
      StateTime stateTime = new StateTime(geometricLayer.getMouseSe2State(), getStateTimeNow().time());
      if (!obstacleQuery_isDisjoint(stateTime))
        color = new Color(255, 96, 96, 128);
      geometricLayer.pushMatrix(geometricLayer.getMouseSe2Matrix());
      graphics.setColor(color);
      graphics.fill(geometricLayer.toPath2D(shape()));
      geometricLayer.popMatrix();
    }
    if (Objects.nonNull(raytraceQuery)) {
      // CAMERA
      Tensor xya = geometricLayer.getMouseSe2State();
      xya = getStateTimeNow().state();
      Se2Bijection se2Bijection = new Se2Bijection(xya);
      TensorUnaryOperator forward = se2Bijection.forward();
      graphics.setColor(new Color(0, 0, 255, 24));
      final int WIDTH = 64;
      BufferedImage bufferedImage = new BufferedImage(WIDTH, WIDTH, BufferedImage.TYPE_INT_ARGB);
      Graphics g = bufferedImage.getGraphics();
      g.setColor(Color.DARK_GRAY);
      g.fillRect(0, 0, WIDTH, WIDTH);
      g.setColor(new Color(192, 255, 192));
      for (int x = 0; x < bufferedImage.getWidth(); ++x) {
        double xn = x / (double) WIDTH;
        double dist = 0.6 + 1.5 * xn + xn * xn;
        for (int y = 0; y < bufferedImage.getHeight(); ++y) {
          Tensor probe = forward.apply(Tensors.vector(dist, (y - WIDTH / 2) * 0.02 * dist));
          if (raytraceQuery.isMember(new StateTime(probe, RealScalar.ZERO))) { // TODO
            // Point2D point2D = geometricLayer.toPoint2D(probe);
            // graphics.fillRect((int) point2D.getX(), (int) point2D.getY(), 2, 2);
          } else {
            g.fillRect(WIDTH - y - 1, WIDTH - x - 1, 1, 1);
          }
        }
      }
      int SCREEN = WIDTH * 2;
      int OFFSET = 20;
      graphics.setColor(Color.GREEN);
      graphics.drawRect(0, OFFSET, SCREEN + 1, SCREEN + 1);
      graphics.drawImage(bufferedImage, 1, OFFSET + 1, SCREEN, SCREEN, null);
    }
  }
}
