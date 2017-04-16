// code by jph
package ch.ethz.idsc.owly.util.rn;

import ch.ethz.idsc.owly.util.ImplicitFunctionRegion;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.AbsSquared;

public class RnSphericalRegion extends ImplicitFunctionRegion {
  public RnSphericalRegion(Tensor center, Scalar radius) {
    super(tensor -> Norm._2Squared.of(center.subtract(tensor)).subtract(AbsSquared.of(radius)));
  }
}
