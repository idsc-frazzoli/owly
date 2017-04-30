// code by jph
package ch.ethz.idsc.owly.math;

import java.util.function.Function;

import ch.ethz.idsc.tensor.Tensor;

public class MappedRegion implements Region {
  final Region region;
  final Function<Tensor, Tensor> function;

  public MappedRegion(Region region, Function<Tensor, Tensor> function) {
    this.region = region;
    this.function = function;
  }

  @Override
  public boolean isMember(Tensor tensor) {
    return region.isMember(function.apply(tensor));
  }
}
