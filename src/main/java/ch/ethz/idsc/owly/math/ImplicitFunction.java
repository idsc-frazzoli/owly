// code by jph
package ch.ethz.idsc.owly.math;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** implicit function f:Tensor -> Scalar
 * 
 * implicit functions define regions via {x | f(x) < 0 or f(x) > 0} */
public interface ImplicitFunction extends Serializable {
  /** @param x
   * @return */
  Scalar evaluate(Tensor x);
}
