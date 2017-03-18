// code by jph
package ch.ethz.idsc.owly.util.rn;

import ch.ethz.idsc.owly.util.ImplicitFunctionRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public class RnSphericalRegion extends ImplicitFunctionRegion {
  public RnSphericalRegion(Tensor center, RealScalar radius) {
    super(tensor -> (RealScalar) //
    Norm._2squared.of(center.subtract(tensor)).minus(radius.absSquared()));
  }
}
