// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

public abstract class ImplicitFunctionRegion implements Region, ImplicitFunction {
  @Override
  public final boolean isMember(Tensor tensor) {
    RealScalar realScalar = (RealScalar) apply(tensor);
    return realScalar.signInt() <= 0;
  }
}
