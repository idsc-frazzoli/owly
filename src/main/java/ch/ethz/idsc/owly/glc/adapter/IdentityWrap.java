// code by jph
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public class IdentityWrap implements CoordinateWrap {
  @Override
  public Tensor represent(Tensor x) {
    return x;
  }

  @Override
  public Scalar distance(Tensor p, Tensor q) {
    return Norm.INFINITY.of(p.subtract(q));
  }
}
