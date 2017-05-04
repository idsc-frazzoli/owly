// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

public class BoundedBoxRegion implements Region {
  private final Tensor lo;
  private final Tensor hi;

  public BoundedBoxRegion(Tensor center, Tensor radius) {
    lo = center.subtract(radius);
    hi = center.add(radius);
  }

  @Override
  public boolean isMember(Tensor tensor) {
    boolean inside = true;
    for (int index = 0; index < tensor.length(); ++index) {
      Scalar value = tensor.Get(index);
      inside &= //
          Scalars.lessEquals(lo.Get(index), value) && //
              Scalars.lessEquals(value, hi.Get(index));
    }
    return inside;
  }
}
