// code by jl
package ch.ethz.idsc.owly.demo.twd;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Tensor;

//TODO Delete?
@Deprecated
public interface MultiVariableLipschitz extends Serializable {
  /** | f(x_1, u) - f(x_2, u) | <= L | x_1 - x_2 |
   * 
   * @return L */
  public Tensor getTensorLipschitz();
}
