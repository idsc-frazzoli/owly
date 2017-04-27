// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;

/** function f:R^n -> R
 * 
 * implicit functions define regions via {x | f(x) < 0 or f(x) > 0} */
public interface ImplicitFunction {
  // ---
  Scalar evaluate(Tensor x);
}
