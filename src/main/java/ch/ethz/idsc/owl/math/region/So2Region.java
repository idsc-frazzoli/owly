// code by jph
package ch.ethz.idsc.owl.math.region;

import ch.ethz.idsc.owl.data.GlobalAssert;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Mod;
import ch.ethz.idsc.tensor.sca.Sign;

public class So2Region extends ImplicitFunctionRegion {
  private final Scalar center;
  private final Scalar radius;
  private final Mod mod;

  public So2Region(Scalar center, Scalar radius) {
    this(center, radius, RealScalar.of(Math.PI));
  }

  public So2Region(Scalar center, Scalar radius, Scalar half_circumference) {
    GlobalAssert.that(Sign.isPositiveOrZero(radius));
    this.center = center;
    this.radius = radius;
    mod = Mod.function(half_circumference.multiply(RealScalar.of(2)), half_circumference.negate());
  }

  @Override // from TensorScalarFunction
  public Scalar apply(Tensor x) {
    return mod.apply(x.Get().subtract(center)).abs().subtract(radius);
  }

  public Scalar center() {
    return center;
  }

  public Scalar radius() {
    return radius;
  }
}
