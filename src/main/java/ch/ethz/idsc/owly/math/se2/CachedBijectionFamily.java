// code by jph
package ch.ethz.idsc.owly.math.se2;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Scalar;

public class CachedBijectionFamily implements BijectionFamily {
  public static BijectionFamily wrap(BijectionFamily bijectionFamily) {
    return bijectionFamily;
  }

  // ---
  private final BijectionFamily bijectionFamily;

  public CachedBijectionFamily(BijectionFamily bijectionFamily) {
    this.bijectionFamily = bijectionFamily;
  }

  @Override
  public TensorUnaryOperator forward(Scalar scalar) {
    // TODO JAN implement
    return bijectionFamily.forward(scalar);
  }

  @Override
  public TensorUnaryOperator inverse(Scalar scalar) {
    // TODO JAN implement
    return bijectionFamily.inverse(scalar);
  }
}
