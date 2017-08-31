// code by jl
package ch.ethz.idsc.owly.glc.adapter;

import ch.ethz.idsc.owly.math.CoordinateWrap;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;

public enum IdentityWrap implements CoordinateWrap {
  INSTANCE;
  // ---
  @Override
  public Tensor represent(Tensor x) {
    return x;
  }

  @Override
  public Scalar distance(Tensor p, Tensor q) {
    return Norm.INFINITY.ofVector(p.subtract(q));
  }
}
