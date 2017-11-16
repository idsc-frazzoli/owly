// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Optional;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.demo.rn.RnRaytracer;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.lie.AngleVector;

public class LidarEmulator implements RenderInterface {
  public static final Scalar SPEED = RealScalar.of(10);
  // ---
  private final Scalar interval;
  private final Supplier<StateTime> supplier;
  private final TrajectoryRegionQuery raytraceQuery;
  private Scalar next;
  // private final Tensor directions = Tensors.empty();
  private Tensor polygon = Tensors.empty();
  private final Tensor sampling;

  /** @param resolution of image in pixels along width and height
   * @param frameRate
   * @param supplier
   * @param raytraceQuery */
  public LidarEmulator( //
      int resolution, Scalar frameRate, //
      Supplier<StateTime> supplier, //
      TrajectoryRegionQuery raytraceQuery) {
    this.interval = frameRate.reciprocal();
    this.supplier = supplier;
    this.raytraceQuery = raytraceQuery;
    sampling = Subdivide.of(+Math.PI / 2, -Math.PI / 2, resolution - 1);
  }

  public Tensor detectRange() {
    StateTime stateTime = supplier.get();
    // if (Objects.isNull(next) || Scalars.lessThan(next, stateTime.time()))
    next = stateTime.time().add(interval);
    RnRaytracer lidarRaytrace = new RnRaytracer(raytraceQuery, SPEED);
    Tensor xya = stateTime.state();
    Tensor origin = xya.extract(0, 2).add(AngleVector.of(xya.Get(2)).multiply(RealScalar.of(.1)));
    Scalar time0 = stateTime.time();
    // TODO not only store distance, but also collision point!
    Tensor range = Tensor.of(sampling.stream().parallel().map(angle -> {
      Optional<StateTime> optional = lidarRaytrace.firstMember( //
          new StateTime(origin, time0), AngleVector.of(xya.Get(2).add(angle)));
      return optional.isPresent() //
          ? optional.get().time().subtract(time0).multiply(SPEED)
          : DoubleScalar.POSITIVE_INFINITY;
    }));
    return range;
  }

  // public Tensor polygon(Tensor range) {
  // StateTime stateTime = supplier.get();
  //// RnRaytracer lidarRaytrace = new RnRaytracer(raytraceQuery, SPEED);
  // Tensor xya = stateTime.state();
  // // Tensor range = Array.zeros(129);
  // Tensor polygon = Tensors.empty();
  // Tensor origin = xya.extract(0, 2).add(AngleVector.of(xya.Get(2)).multiply(RealScalar.of(.1)));
  // polygon.append(origin);
  // Scalar time0 = stateTime.time();
  // int index = 0;
  // for (Tensor angle : sampling)
  // if (range.Get(index)) {
  // Optional<StateTime> optional = lidarRaytrace.firstMember( //
  // );
  // if (optional.isPresent()) {
  // // polygon.append(optional.get().state());
  // Scalar tof = optional.get().time().subtract(time0).multiply(SPEED);
  // range.set(tof, index);
  // } else
  // range.set(RealScalar.ZERO, index);
  // ++index;
  // }
  // return polygon;
  // }
  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    {
      // TODO scale so that range limit == image height
      Tensor range = detectRange();
      graphics.setColor(new Color(224, 224, 255, 128));
      graphics.fillRect(0, 200, range.length(), 100);
      graphics.setColor(Color.RED);
      for (int index = 0; index < range.length(); ++index)
        graphics.fillRect(index, (int) (300 - range.Get(index).number().doubleValue() * 20), 1, 2);
    }
    // {
    // Path2D path2D = geometricLayer.toPath2D(polygon);
    // graphics.setColor(new Color(0, 255, 0, 16));
    // graphics.fill(path2D);
    // path2D.closePath();
    // graphics.setColor(new Color(0, 255, 0, 64));
    // graphics.draw(path2D);
    // }
  }
}
