// code by jph
package ch.ethz.idsc.owly.demo.se2;

import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public class Se2IdentityWrap implements CoordinateWrap {
  // TODO still needs weights to unify x-y ["meters"] with angle [radians]
  @Override
  public Tensor represent(Tensor x) {
    return x;
  }

  @Override
  public Scalar distance(Tensor p, Tensor q) {
    return Norm.Infinity.of(p.subtract(q));
  }
}
