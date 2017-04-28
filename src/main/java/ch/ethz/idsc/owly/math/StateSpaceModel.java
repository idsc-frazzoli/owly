// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** system dynamics described by a differential constraint
 * 
 * inspired by
 * <a href="https://reference.wolfram.com/language/ref/StateSpaceModel.html">StateSpaceModel</a> */
public interface StateSpaceModel {
  /** flow is function f in
   * (d_t x) |_t == f(x(t), u(t))
   * 
   * @param x
   * @param u
   * @return */
  Tensor createFlow(Tensor x, Tensor u);

  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 |
   * 
   * @return L */
  Scalar getLipschitz();
}
