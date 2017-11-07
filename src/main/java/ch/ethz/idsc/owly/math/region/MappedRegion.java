// code by jph
package ch.ethz.idsc.owly.math.region;

import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.tensor.Tensor;

/** member check is carried on the input tensor mapped by the given operator */
public class MappedRegion implements TensorRegion {
  private final TensorRegion region;
  private final TensorUnaryOperator function;

  public MappedRegion(TensorRegion region, TensorUnaryOperator function) {
    this.region = region;
    this.function = function;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return region.isMember(function.apply(tensor));
  }
}
