// code by jph
package ch.ethz.idsc.owl.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.alg.Join;

/** the name "double" hints that if the state is (position, velocity) then
 * control u acts as (acceleration).
 * 
 * implementation for linear coordinate system R^n
 * 
 * see also {@link SingleIntegratorStateSpaceModel} */
public enum DoubleIntegratorStateSpaceModel implements StateSpaceModel {
  INSTANCE;
  // ---
  /** f((p, v), u) == (v, u) */
  @Override
  public Tensor f(Tensor x, Tensor u) {
    if (x.length() != u.length() << 1)
      throw TensorRuntimeException.of(x, u);
    Tensor v = x.extract(u.length(), x.length());
    return Join.of(v, u);
  }

  @Override
  public Scalar getLipschitz() {
    return RealScalar.ONE;
  }
}
