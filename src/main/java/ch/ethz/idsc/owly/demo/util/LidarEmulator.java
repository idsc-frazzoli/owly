// code by jph
package ch.ethz.idsc.owly.demo.util;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.Degree;
import ch.ethz.idsc.owly.math.se2.Se2Bijection;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.owly.math.state.TrajectoryRegionQuery;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Subdivide;
import ch.ethz.idsc.tensor.lie.AngleVector;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;
import ch.ethz.idsc.tensor.red.Norm;

public class LidarEmulator implements RenderInterface {
  public static final Tensor DEFAULT = Subdivide.of(Degree.of(+90), Degree.of(-90), 64);
  public static final Scalar RANGE_MAX = RealScalar.of(5.0);
  // ---
  private final Scalar interval;
  private final Supplier<StateTime> supplier;
  private final TrajectoryRegionQuery raytraceQuery;
  @SuppressWarnings("unused")
  private Scalar next;
  private final Tensor directions;
  private final List<Tensor> localRays = new ArrayList<>();

  /** @param resolution angular resolution (should be tensor)
   * @param frameRate
   * @param supplier
   * @param raytraceQuery */
  public LidarEmulator( //
      Tensor sampling, Scalar frameRate, //
      Supplier<StateTime> supplier, //
      TrajectoryRegionQuery raytraceQuery) {
    this.interval = frameRate.reciprocal();
    this.supplier = supplier;
    this.raytraceQuery = raytraceQuery;
    // ---
    directions = Tensor.of(sampling.stream().map(Scalar.class::cast).map(AngleVector::of));
    for (Tensor dir : directions)
      localRays.add(Tensor.of(Subdivide.of(RealScalar.ZERO, RANGE_MAX, 60).stream() // magic const
          .map(Scalar.class::cast) //
          .map(dir::multiply)));
  }

  public Tensor detectRange() {
    StateTime stateTime = supplier.get();
    // if (Objects.isNull(next) || Scalars.lessThan(next, stateTime.time()))
    next = stateTime.time().add(interval);
    Scalar time = stateTime.time();
    Se2Bijection se2Bijection = new Se2Bijection(stateTime.state());
    TensorUnaryOperator forward = se2Bijection.forward();
    // Stopwatch stopwatch = Stopwatch.started();
    Tensor range = Tensor.of(localRays.stream().parallel() //
        .map(rays -> {
          Optional<Tensor> first = rays.stream() //
              .filter(local -> raytraceQuery.isMember(new StateTime(forward.apply(local), time))) //
              .findFirst();
          return first.isPresent() ? Norm._2.ofVector(first.get()) : RANGE_MAX;
        }));
    // RnRaytracer lidarRaytrace = new RnRaytracer(raytraceQuery, SPEED);
    // Tensor xya = stateTime.state();
    // Tensor origin = xya.extract(0, 2).add(AngleVector.of(xya.Get(2)).multiply(RealScalar.of(.1)));
    // Scalar time0 = stateTime.time();
    // // not only store distance, but also collision point!
    // Tensor range = Tensor.of(sampling.stream().map(angle -> {
    // Optional<StateTime> optional = lidarRaytrace.firstMember( //
    // new StateTime(origin, time0), AngleVector.of(xya.Get(2).add(angle)));
    // return optional.isPresent() //
    // ? optional.get().time().subtract(time0).multiply(SPEED)
    // : DoubleScalar.POSITIVE_INFINITY;
    // }));
    // System.out.println(stopwatch.display_seconds());
    return range;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Tensor range = detectRange();
    // {
    // graphics.setColor(new Color(224, 224, 255, 128));
    // graphics.fillRect(0, 200, range.length(), 100);
    // graphics.setColor(Color.RED);
    // Clip clip = Clip.function(RealScalar.ZERO, RANGE_MAX);
    // for (int index = 0; index < range.length(); ++index) {
    // graphics.fillRect(index, 300 - (int) (100 * clip.rescale(range.Get(index)).number().doubleValue()), 1, 2);
    // }
    // }
    {
      StateTime stateTime = supplier.get();
      Se2Bijection se2Bijection = new Se2Bijection(stateTime.state());
      TensorUnaryOperator forward = se2Bijection.forward();
      Tensor polygon = Tensor.of(range.pmul(directions).stream().map(forward));
      polygon.append(forward.apply(Array.zeros(2)));
      Path2D path2D = geometricLayer.toPath2D(polygon);
      graphics.setColor(new Color(0, 255, 0, 16));
      graphics.fill(path2D);
      path2D.closePath();
      graphics.setColor(new Color(0, 255, 0, 64));
      graphics.draw(path2D);
    }
  }
}
