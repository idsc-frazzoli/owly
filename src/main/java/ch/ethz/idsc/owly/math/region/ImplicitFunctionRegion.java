// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.math.ImplicitFunction;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/** region {x | f(x) <= 0} defined by the overriding implicit function f
 * 
 * for instance, the function f can be the distance to
 * and obstacle:
 * <ul>
 * <li>positive when outside the obstacle,
 * <li>zero when touching the obstacle, and
 * <li>negative when in collision
 * </ul> */
public abstract class ImplicitFunctionRegion implements Region, ImplicitFunction {
  @Override
  public final boolean isMember(Tensor tensor) {
    RealScalar realScalar = (RealScalar) evaluate(tensor);
    return realScalar.signInt() <= 0;
  }
}
