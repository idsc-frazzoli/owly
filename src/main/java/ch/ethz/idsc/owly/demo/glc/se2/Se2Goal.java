// code by jph
package ch.ethz.idsc.owly.demo.glc.se2;

import ch.ethz.idsc.owly.glc.core.Heuristic;
import ch.ethz.idsc.owly.math.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

public class Se2Goal implements Region, Heuristic {
  static final Mod PRINCIPAL = Mod.function(RealScalar.of(2 * Math.PI), RealScalar.of(-Math.PI));
  // ---
  final Tensor xy;
  final Scalar angle;
  final Scalar radius;
  final Scalar angle_delta;

  public Se2Goal(Tensor xy, Scalar angle, Scalar radius, Scalar angle_delta) {
    this.xy = xy;
    this.angle = angle;
    this.radius = radius;
    this.angle_delta = angle_delta;
  }

  @Override
  public Scalar costToGoal(Tensor x) {
    Tensor cur_xy = x.extract(0, 2);
    Scalar cur_angle = x.Get(2);
    Scalar dxy = Norm._2.of(cur_xy.subtract(xy)).subtract(radius);
    // Scalar dangle = PRINCIPAL.apply(cur_angle.subtract(angle)).abs().subtract(angle_delta);
    return Max.of(dxy, ZeroScalar.get());
    // return Max.of(Norm._2.of(tensor.subtract(center)).subtract(radius), ZeroScalar.get());
    // return ZeroScalar.get();
  }

  @Override
  public boolean isMember(Tensor tensor) {
    Tensor cur_xy = tensor.extract(0, 2);
    Scalar cur_angle = tensor.Get(2);
    boolean status = true;
    status &= Scalars.lessEquals(Norm._2.of(cur_xy.subtract(xy)), radius);
    status &= Scalars.lessEquals(PRINCIPAL.apply(cur_angle.subtract(angle)).abs(), angle_delta);
    return status;
  }
}