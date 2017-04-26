// code by jph
package ch.ethz.idsc.owly.math;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensor;

/** region {x | f(x) <= 0} defined by the overriding implicit function f
 * 
 * for instance, the function f can be the distance to
 * and obstacle: positive when outside the obstacle,
 * zero when touching the obstacle, and negative when in collision */
public abstract class ImplicitFunctionRegion implements Region, ImplicitFunction {
  @Override
  public final boolean isMember(Tensor tensor) {
    RealScalar realScalar = (RealScalar) apply(tensor);
    return realScalar.signInt() <= 0;
  }
}
