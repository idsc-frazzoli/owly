// code by jph
package ch.ethz.idsc.owly.demo.se2.glc;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.owly.demo.rn.RnRaytracer;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.ani.AbstractEntity;
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
    if (Objects.nonNull(raytraceQuery) && false) { // LIDAR
      StateTime stateTime = getStateTimeNow();
      Scalar speed = RealScalar.of(10);
      RnRaytracer lidarRaytrace = new RnRaytracer(raytraceQuery, speed);
      Tensor xya = stateTime.state();
      graphics.setColor(new Color(255, 0, 0, 32));
      Tensor range = Array.zeros(129);
      int index = 0;
      Tensor polygon = Tensors.empty();
      Tensor origin = xya.extract(0, 2).add(AngleVector.of(xya.Get(2)).multiply(RealScalar.of(.1)));
      polygon.append(origin);
      Scalar time0 = stateTime.time();
      for (Tensor angle : Subdivide.of(+Math.PI / 2, -Math.PI / 2, range.length() - 1)) {
        // Subdivide.of(+Math.PI / 2, -Math.PI / 2, range.length() - 1).stream().parallel().map(angle->{
        Optional<StateTime> optional = lidarRaytrace.firstMember( //
            new StateTime(origin, time0), AngleVector.of(xya.Get(2).add(angle)));
        if (optional.isPresent()) {
          polygon.append(optional.get().state());
          Scalar tof = optional.get().time().subtract(time0).multiply(speed);
          range.set(tof, index);
        }
        ++index;
        // return RealScalar.ZERO;
      }
      // );
      {
        Path2D path2D = geometricLayer.toPath2D(polygon);
        graphics.setColor(new Color(0, 255, 0, 16));
        graphics.fill(path2D);
        path2D.closePath();
        graphics.setColor(new Color(0, 255, 0, 64));
        graphics.draw(path2D);
      }
      graphics.setColor(new Color(224, 224, 255, 128));
      graphics.fillRect(0, 200, range.length(), 100);
      graphics.setColor(Color.RED);
      for (index = 0; index < range.length(); ++index)
        graphics.fillRect(index, (int) (300 - range.Get(index).number().doubleValue() * 10), 1, 2);
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
  }
}
