// code by jph
package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public enum Se2Exp {
  ;
  /** maps an element (x, y, t) of the lie-algebra se2 in standard coordinates:
   * [0 -t x]
   * [t 0 y]
   * [0 0 0]
   * to the corresponding element in SE2.
   * 
   * @param x
   * @param y
   * @param theta
   * @return */
  public static Tensor of(Tensor x) {
    return Se2Utils.toSE2Matrix(Se2Integrator.combine(Tensors.vector(0, 0, 0), x));
  }
}
