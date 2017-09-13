// code by jph
package ch.ethz.idsc.owly.demo.drift;

import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;

/** open region consisting of all states with a negative coordinate at a given index */
public class NegativeHalfspaceRegion implements Region {
  private final int index;

  /** @param index of state coordinate that when negative is inside the region */
  public NegativeHalfspaceRegion(int index) {
    this.index = index;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return Scalars.lessThan(tensor.Get(index), RealScalar.ZERO);
  }
}
