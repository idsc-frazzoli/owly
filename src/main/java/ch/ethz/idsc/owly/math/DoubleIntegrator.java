// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Join;

/** implementation for R^n */
public enum DoubleIntegrator implements StateSpaceModel {
  INSTANCE;
  // ---
  /** f((x, v), u) == (v, u) */
  @Override
  public Tensor f(Tensor x, Tensor u) {
    if (x.length() != u.length() * 2)
      throw new RuntimeException();
    Tensor v = x.extract(u.length(), x.length());
    return Join.of(v, u);
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
