// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.sca.Mod;
import ch.ethz.idsc.tensor.sca.Sign;

public class So2Region extends ImplicitFunctionRegion {
  private static final Mod PRINCIPAL = Mod.function(2 * Math.PI, -Math.PI);
  // ---
  private final Scalar center;
  private final Scalar radius;

  public So2Region(Scalar center, Scalar radius) {
    GlobalAssert.that(Sign.isPositiveOrZero(radius));
    this.center = center;
    this.radius = radius;
  }

  @Override // from ImplicitFunction
  public Scalar evaluate(Tensor x) {
    return PRINCIPAL.apply(x.Get().subtract(center)).abs().subtract(radius);
  }

  public Scalar center() {
    return center;
  }

  public Scalar radius() {
    return radius;
  }
}
