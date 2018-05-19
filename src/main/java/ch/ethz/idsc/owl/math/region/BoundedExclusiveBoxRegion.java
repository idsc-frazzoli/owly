// code by jph, ynager
package ch.ethz.idsc.owl.math.region;

import java.util.stream.IntStream;

import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** region is open
 * coordinates on the boundary are inside
 * same convention as {@link ImplicitFunctionRegion} */
public class BoundedExclusiveBoxRegion implements Region<Tensor> {
  private final Tensor lo;
  private final Tensor hi;

  public BoundedExclusiveBoxRegion(Tensor center, Tensor radius) {
    lo = center.subtract(radius);
    hi = center.add(radius);
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return IntStream.range(0, lo.length()) //
        .allMatch(index -> Scalars.lessThan(lo.Get(index), tensor.Get(index)) //
            && Scalars.lessThan(tensor.Get(index), hi.Get(index)));
  }
}
