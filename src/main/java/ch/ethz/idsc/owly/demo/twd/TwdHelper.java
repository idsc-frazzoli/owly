// code by jl
package ch.ethz.idsc.owly.demo.twd;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.Mod;

public enum TwdHelper {
  ;
  private static final Mod PRINCIPAL = Mod.function(2 * Math.PI, -Math.PI);

  /***************************************************/
  /** one application of the function is as a heuristic
   * 
   * @param state1 = {px1, py1, theta1}
   * @param state2 = {px2, py2, theta2}
   * @return non-negative positional distance between state1 and state2 */
  public static Scalar errorPosition(Tensor state1, Tensor state2) {
    return Norm._2.between(state1.extract(0, 2), state2.extract(0, 2));
  }

  public static Scalar errorRotation(Tensor state1, Tensor state2) {
    return PRINCIPAL.apply(state1.Get(2).subtract(state2.Get(2))).abs();
  }
}
