// code by jph
package ch.ethz.idsc.owly.util;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.ZeroScalar;

public class ImplicitFunctionRegion implements Region {
  private final ImplicitFunction implicitFunction;

  public ImplicitFunctionRegion(ImplicitFunction implicitFunction) {
    this.implicitFunction = implicitFunction;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return 0 <= ZeroScalar.get().compareTo(implicitFunction.apply(tensor));
  }
}
